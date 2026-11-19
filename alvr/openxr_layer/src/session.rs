//! Sessions, the layer's own reference spaces in them, and ALVR recentering.
//!
//! ALVR recenters by moving the space it submits poses to SteamVR in, which SteamVR does not see:
//! to applications, every reference space but VIEW moves without the
//! XrEventDataReferenceSpaceChangePending a runtime must queue. The driver counts these moves and
//! sends them with the markers, and the layer queues the event for every reference space of every
//! session but VIEW.

use crate::{Downstream, State, log, math, state, void_fn};
use alvr_marker_ipc::{self as ipc, Capabilities, Pose, ns_to_performance_counter};
use openxr_sys::{self as xr, Handle, pfn};
use std::{collections::VecDeque, ffi::CStr, ptr};

pub(crate) struct SessionData {
    pub instance: u64,
    /// Reference spaces of the layer's own, at their natural origins, created when first needed.
    spaces: Vec<(xr::ReferenceSpaceType, xr::Space)>,
    /// The reference space types of the session, once asked for.
    reference_space_types: Option<Vec<xr::ReferenceSpaceType>>,
    /// Origin changes not yet reported.
    origin_changes: VecDeque<OriginChange>,
    /// Reference space change events ready for xrPollEvent.
    space_events: VecDeque<SpaceEvent>,
}

impl SessionData {
    fn new(instance: u64) -> Self {
        Self {
            instance,
            spaces: Vec::new(),
            reference_space_types: None,
            origin_changes: VecDeque::new(),
            space_events: VecDeque::new(),
        }
    }
}

/// ALVR moved the space it submits poses to SteamVR in: for applications, every reference space
/// but VIEW moved by as much.
#[derive(Clone, Copy, Debug)]
pub(crate) struct OriginChange {
    /// When it took effect, on the performance counter clock.
    pub changed_ns: i64,
    /// Where the new origin of STAGE is in the previous STAGE, when known.
    pub pose_in_previous: Option<Pose>,
    /// False when only the bounds of STAGE changed.
    pub moved: bool,
}

impl OriginChange {
    /// How the space the poses are in moved from the origin the driver sent before to `origin`,
    /// if it did.
    pub fn between(previous: ipc::OriginChange, origin: ipc::OriginChange) -> Option<Self> {
        if origin.count == previous.count {
            return None;
        }

        // A step from the last change seen, or else (a missed one, a restarted driver) unknown
        let follows = origin.count == previous.count + 1;
        Some(Self {
            changed_ns: origin.changed_ns,
            pose_in_previous: origin.pose_in_previous.filter(|_| follows),
            // A missed change may have moved it
            moved: origin.moved || !follows,
        })
    }

    /// This change, then `next`.
    fn then(self, next: Self) -> Self {
        Self {
            changed_ns: next.changed_ns,
            pose_in_previous: self
                .pose_in_previous
                .zip(next.pose_in_previous)
                .map(|(first, second)| math::compose(first, second)),
            moved: self.moved || next.moved,
        }
    }
}

/// An XrEventDataReferenceSpaceChangePending of a session, without the `next` pointer that would
/// keep it out of the layer's state, which is shared between threads.
#[derive(Clone, Copy, Debug)]
struct SpaceEvent {
    reference_space_type: xr::ReferenceSpaceType,
    change_time: xr::Time,
    /// Where the new origin is in the previous space, when known.
    pose_in_previous_space: Option<Pose>,
}

impl SpaceEvent {
    fn into_event(self, session: u64) -> xr::EventDataReferenceSpaceChangePending {
        xr::EventDataReferenceSpaceChangePending {
            ty: xr::EventDataReferenceSpaceChangePending::TYPE,
            next: ptr::null(),
            session: xr::Session::from_raw(session),
            reference_space_type: self.reference_space_type,
            change_time: self.change_time,
            pose_valid: self.pose_in_previous_space.is_some().into(),
            pose_in_previous_space: self
                .pose_in_previous_space
                .map_or(xr::Posef::IDENTITY, math::to_xr),
        }
    }
}

pub(crate) fn function(name: &CStr) -> Option<pfn::VoidFunction> {
    Some(match name.to_bytes() {
        b"xrCreateSession" => void_fn!(create_session, pfn::CreateSession),
        b"xrDestroySession" => void_fn!(destroy_session, pfn::DestroySession),
        b"xrPollEvent" => void_fn!(poll_event, pfn::PollEvent),
        _ => return None,
    })
}

/// Where `space` is in `base` at `time`. `Ok(None)` when the runtime cannot tell.
pub(crate) unsafe fn locate(
    downstream: &Downstream,
    space: xr::Space,
    base: xr::Space,
    time: xr::Time,
) -> Result<Option<xr::Posef>, xr::Result> {
    let mut location = xr::SpaceLocation {
        ty: xr::SpaceLocation::TYPE,
        next: ptr::null_mut(),
        location_flags: xr::SpaceLocationFlags::EMPTY,
        pose: xr::Posef::IDENTITY,
    };
    let result = unsafe { (downstream.locate_space)(space, base, time, &mut location) };
    if result != xr::Result::SUCCESS {
        return Err(result);
    }

    let valid = xr::SpaceLocationFlags::ORIENTATION_VALID | xr::SpaceLocationFlags::POSITION_VALID;
    Ok(location
        .location_flags
        .contains(valid)
        .then_some(location.pose))
}

/// The reference space types of the session, from the runtime.
unsafe fn reference_space_types(
    downstream: &Downstream,
    session: u64,
) -> Vec<xr::ReferenceSpaceType> {
    let session = xr::Session::from_raw(session);
    let mut count = 0;
    if unsafe { (downstream.enumerate_reference_spaces)(session, 0, &mut count, ptr::null_mut()) }
        != xr::Result::SUCCESS
    {
        return Vec::new();
    }
    let mut types = vec![xr::ReferenceSpaceType::VIEW; count as usize];
    if unsafe {
        (downstream.enumerate_reference_spaces)(session, count, &mut count, types.as_mut_ptr())
    } != xr::Result::SUCCESS
    {
        return Vec::new();
    }
    types.truncate(count as usize);

    types
}

impl State {
    /// Takes in what the driver sent, and hands origin changes to the sessions there are now.
    /// Returns what the headset connected last can do, or `None` while the driver is not running.
    pub(crate) fn refresh_markers(&mut self) -> Option<Capabilities> {
        let (capabilities, changes) = self.markers.refresh();
        for change in changes {
            log!("ALVR recentered: {change:?}");
            for session in self.sessions.values_mut() {
                session.origin_changes.push_back(change);
            }
        }

        capabilities
    }

    pub(crate) fn remove_session(&mut self, session: u64) -> Option<SessionData> {
        let data = self.sessions.remove(&session)?;
        self.remove_session_contexts(session);

        Some(data)
    }

    /// The layer's own reference space of the type, at its natural origin.
    pub(crate) fn reference_space(
        &mut self,
        session: u64,
        ty: xr::ReferenceSpaceType,
    ) -> Option<xr::Space> {
        let data = self.sessions.get(&session)?;
        if let Some(&(_, space)) = data.spaces.iter().find(|(t, _)| *t == ty) {
            return Some(space);
        }
        let downstream = self.downstream(data.instance)?;

        let info = xr::ReferenceSpaceCreateInfo {
            ty: xr::ReferenceSpaceCreateInfo::TYPE,
            next: ptr::null(),
            reference_space_type: ty,
            pose_in_reference_space: xr::Posef::IDENTITY,
        };
        let mut space = xr::Space::NULL;
        let result = unsafe {
            (downstream.create_reference_space)(xr::Session::from_raw(session), &info, &mut space)
        };
        if result != xr::Result::SUCCESS {
            log!("Cannot create a {ty:?} space: {result:?}");
            return None;
        }
        self.sessions.get_mut(&session)?.spaces.push((ty, space));

        Some(space)
    }

    /// A reference space change event for a session of `instance`, if one is due.
    fn take_space_event(
        &mut self,
        instance: u64,
    ) -> Option<xr::EventDataReferenceSpaceChangePending> {
        self.refresh_markers();

        let sessions = self
            .sessions
            .iter()
            .filter(|(_, data)| data.instance == instance)
            .map(|(&session, _)| session)
            .collect::<Vec<_>>();
        for session in sessions {
            loop {
                let data = self.sessions.get_mut(&session)?;
                if let Some(event) = data.space_events.pop_front() {
                    return Some(event.into_event(session));
                }
                // Changes the application has not heard of yet are one change to it
                let mut change = data.origin_changes.pop_front()?;
                while let Some(next) = data.origin_changes.pop_front() {
                    change = change.then(next);
                }
                let events = self.space_events(session, change);
                self.sessions.get_mut(&session)?.space_events.extend(events);
            }
        }

        None
    }

    /// The events for an origin change: one per reference space type of the session but VIEW,
    /// each with where its new origin is in its previous self.
    fn space_events(&mut self, session: u64, change: OriginChange) -> Vec<SpaceEvent> {
        let instance = self.sessions[&session].instance;
        let Some(downstream) = self.downstream(instance) else {
            return Vec::new();
        };
        // The layer's times are the performance counter's; without the conversion, there is no
        // change time to report
        let Some(convert) = downstream.convert_counter else {
            return Vec::new();
        };
        let counter = ns_to_performance_counter(change.changed_ns);
        let mut change_time = xr::Time::from_nanos(0);
        if unsafe { convert(xr::Instance::from_raw(instance), &counter, &mut change_time) }
            != xr::Result::SUCCESS
        {
            return Vec::new();
        }

        let types = match &self.sessions[&session].reference_space_types {
            Some(types) => types.clone(),
            None => {
                let types = unsafe { reference_space_types(&downstream, session) };
                self.sessions
                    .get_mut(&session)
                    .unwrap()
                    .reference_space_types = Some(types.clone());
                types
            }
        };

        types
            .into_iter()
            // Only STAGE has bounds
            .filter(|&ty| {
                ty != xr::ReferenceSpaceType::VIEW
                    && (change.moved || ty == xr::ReferenceSpaceType::STAGE)
            })
            .map(|ty| {
                // The spaces of SteamVR keep their poses relative to each other: each moved like
                // STAGE, seen from its own origin
                let pose = change.pose_in_previous.and_then(|stage_change| {
                    if ty == xr::ReferenceSpaceType::STAGE {
                        return Some(stage_change);
                    }
                    let space = self.reference_space(session, ty)?;
                    let stage = self.reference_space(session, xr::ReferenceSpaceType::STAGE)?;
                    let in_stage = unsafe { locate(&downstream, space, stage, change_time) }
                        .ok()
                        .flatten()
                        .map(math::from_xr)?;
                    Some(math::compose(
                        math::inverse(in_stage),
                        math::compose(stage_change, in_stage),
                    ))
                });

                SpaceEvent {
                    reference_space_type: ty,
                    change_time,
                    pose_in_previous_space: pose,
                }
            })
            .collect()
    }
}

unsafe extern "system" fn create_session(
    instance: xr::Instance,
    info: *const xr::SessionCreateInfo,
    session: *mut xr::Session,
) -> xr::Result {
    let Some(downstream) = state().downstream(instance.into_raw()) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };

    let result = unsafe { (downstream.create_session)(instance, info, session) };
    if result.into_raw() >= 0 {
        state().sessions.insert(
            unsafe { *session }.into_raw(),
            SessionData::new(instance.into_raw()),
        );
    }

    result
}

unsafe extern "system" fn destroy_session(session: xr::Session) -> xr::Result {
    let (data, downstream) = {
        let mut state = state();
        let Some(data) = state.remove_session(session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        state.sync_requests();
        let Some(downstream) = state.downstream(data.instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        (data, downstream)
    };

    for (_, space) in data.spaces {
        unsafe { (downstream.destroy_space)(space) };
    }

    unsafe { (downstream.destroy_session)(session) }
}

unsafe extern "system" fn poll_event(
    instance: xr::Instance,
    event: *mut xr::EventDataBuffer,
) -> xr::Result {
    let Some(downstream) = state().downstream(instance.into_raw()) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };

    let result = unsafe { (downstream.poll_event)(instance, event) };
    if result != xr::Result::EVENT_UNAVAILABLE || event.is_null() {
        return result;
    }

    // The runtime's events come first, so the layer's never delay them
    let mut state = state();
    if let Some(space_event) = state.take_space_event(instance.into_raw()) {
        unsafe {
            event
                .cast::<xr::EventDataReferenceSpaceChangePending>()
                .write(space_event)
        };
        return xr::Result::SUCCESS;
    }
    let Some(context) = state.take_discovery_recommendation(instance.into_raw()) else {
        return result;
    };
    unsafe {
        event
            .cast::<xr::EventDataSpatialDiscoveryRecommendedEXT>()
            .write(xr::EventDataSpatialDiscoveryRecommendedEXT {
                ty: xr::EventDataSpatialDiscoveryRecommendedEXT::TYPE,
                next: ptr::null(),
                spatial_context: xr::SpatialContextEXT::from_raw(context),
            })
    };

    xr::Result::SUCCESS
}
