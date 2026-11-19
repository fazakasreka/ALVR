# ALVR OpenXR marker layer

An implicit OpenXR API layer that gives **any** OpenXR application running on SteamVR + ALVR the QR
codes the headset sees, through the standard `XR_EXT_spatial_entity` and
`XR_EXT_spatial_marker_tracking` extensions.

SteamVR's OpenXR runtime implements neither extension, and an OpenVR driver cannot add OpenXR
extensions. So the layer, loaded by the OpenXR loader into each application, implements them on top
of the runtime:

```text
headset ── QR codes + time, per tracker ─────▶ ALVR driver in vrserver.exe
        ◀─ trackers to run (marker options) ──     │ ▲
                                    markers + time │ │ marker options of the
                     (named pipe, alvr_marker_ipc) ▼ │ application's contexts
app ─▶ OpenXR loader ─▶ this layer ─▶ SteamVR OpenXR runtime
```

- **The headset tracks every frame.** It finds codes with a discovery every 500 ms and follows them
  with an update snapshot every frame, at the frame's display time. It runs one tracker per marker
  configuration (static or not, known size or not) that applications ask for, as far as it
  supports the options.
- **The driver owns the markers.** It recenters them like the headset pose, converts their time to
  the PC's performance counter the way the headset pose's time is, and serves them on the named
  pipe `\\.\pipe\ALVR_Markers` (see `alvr/marker_ipc`): each connected layer gets the complete
  current state when it connects and whenever it changes. Poses are in the space the driver submits
  the headset pose in, which SteamVR's OpenXR runtime calls `STAGE` (the driver keeps the standing
  zero pose at identity). It collects the marker options of all layers and has the headset run them.
- **The layer answers for any time.** It reads whatever has arrived when the application asks for
  markers, without waiting, keeps a second of poses, and maps them onto the extensions: it converts
  the requested time with `XR_KHR_win32_convert_performance_counter_time` (enabling it on the
  runtime), and locates its own `STAGE` space in the application's base space at that time. A
  broken pipe means the driver is gone.

## What is implemented

| | |
|---|---|
| Capabilities | `XR_SPATIAL_CAPABILITY_MARKER_TRACKING_QR_CODE_EXT`, when the headset connected last tracks QR codes |
| Components | `MARKER` (the code's data: a string buffer for text, a `uint8` buffer for other bytes), `BOUNDED_2D` (centre pose and size) |
| Features | fixed size markers, static markers; both always honoured, see below |
| Snapshots | discovery and update snapshots, `XrSpatialFilterTrackingStateEXT` |
| Poses | at the requested time, filtered and predicted up to 100 ms past the latest detection; see **Jitter** |
| Entities | a context gives a code an id when it first discovers it and keeps it, so it is stable across the context's snapshots; no two contexts share an id, as the extension requires. Buffer ids come from the code's data |
| Tracking states | `TRACKING` while the headset sees the code; `PAUSED` otherwise, while disconnected, or when nothing came for 500 ms; never `STOPPED` |
| Data | up to 7089 bytes, the most a QR code holds; codes the headset has not decoded yet are not reported |
| Events | `XrEventDataSpatialDiscoveryRecommendedEXT` when a context is created and when the set of codes changes |
| `XR_EXT_future` | provided when the runtime lacks it; futures of the layer are ready immediately |

ArUco, AprilTag and Micro QR are reported as unsupported: they are not enumerated, and their
capability gets `XR_ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT`. The headset only reports QR codes.
An instance that enables `XR_EXT_spatial_entity` without `XR_EXT_spatial_marker_tracking` has no
capability at all.

**Marker options.** A context's `XrSpatialMarkerSizeEXT` and `XrSpatialMarkerStaticOptimizationEXT`
go to the headset, which uses them for more precise poses when it supports them. The layer honours
them either way, as the extension requires: a context with a size gets every code at that size, and
a context for static codes gets their poses averaged over the last second instead of predicted.

**Jitter.** Detections jitter by millimetres and fractions of a degree, and now and then one is off
by centimetres. Applications ask for the display time, tens of milliseconds past the latest
detection, so a prediction from raw detections magnifies the noise. The layer (`src/history.rs`):

- filters moving codes with a One Euro filter, on position and orientation: strongly while they
  stand still, little while they move fast. The filter's lag is made up for, and the prediction
  uses velocities fitted over the latest 200 ms of detections, both fading out near standstill
  so that noise is not extrapolated.
- averages static codes without the detections far from the rest.
- holds back a detection far from the prediction until the next one confirms it: a single wild
  detection is ignored, a real jump is followed one detection late.

With 2 mm and 0.5° of noise per axis and 1% of detections 3 cm off, at 60 Hz, 40 ms ahead
(`cargo test -p alvr_openxr_layer jitter::report -- --nocapture`):

| | Before | Now |
|---|---|---|
| Still code, moving context | 20.2 mm, 3.8° | 0.9 mm, 0.20° |
| Walking at 0.5 m/s | 20.2 mm, 3.8° | 2.6 mm, 0.20° |
| Circling (the emulator's dog) | 20.2 mm, 3.8° | 3.8 mm, 0.60° |
| Standing, walking, stopping | 20.3 mm, 3.8° | 7.3 mm, 0.64° |
| Still code, static context | 0.7 mm, 0.10° | 0.4 mm, 0.10° |

Root mean square errors against the truth. A code that stops is still predicted on for a few
frames, about its speed times the prediction time: no predictor can know of the stop.
`jitter_stays_small` keeps these from getting worse.

**Recentering.** ALVR recenters by moving the space it submits poses to SteamVR in, which SteamVR
does not see: to applications, all their reference spaces move without the
`XrEventDataReferenceSpaceChangePending` a runtime must queue. So the driver counts these moves and
sends them with the markers, and the layer queues the event for every reference space of every
session but `VIEW`, in every application, marker extensions or not:

- `changeTime` is when the driver started submitting poses in the new space.
- ALVR moves it when the recentering mode changes while streaming, and when the headset's own
  STAGE space changes (a new boundary, or a recenter that moves STAGE). The headset reports the
  change its runtime gives in its own `XrEventDataReferenceSpaceChangePending`, with plain OpenXR,
  so on any headset. The modes but Stage then recenter on the head, in the headset's new space.
- `poseInPreviousSpace` is ALVR's move combined with the headset's: valid when both are known. It
  is not known for a new connection, or when the headset's runtime does not say where STAGE went
  (`poseValid` false there).
- A change that leaves the origin where it was is not reported: in the Stage mode, a recenter that
  only moves the headset's LOCAL space changes nothing for applications.
- A new size of the play area is reported for `STAGE` alone, with the origin where it was. SteamVR
  does not report it itself.
- Changes that come before the application polls are reported as one.
- Marker poses kept from before the move are moved with it, or forgotten when it is not known.

Sessions created after a move do not get its event.

**Coexistence.** The layer first creates the instance with the extensions the application asked
for. If the runtime accepts them, it implements them itself and the layer steps aside. An
application that enables none of the extensions talks to the runtime directly, but for
`xrCreateSession`, `xrDestroySession` and `xrPollEvent`, which report recentering, and
`XR_KHR_win32_convert_performance_counter_time`, which the layer enables on the runtime for the
change times.

## Conformance

The OpenXR CTS (1.1.65) passes `XR_EXT_spatial_marker_tracking-qr-code`, also with invalid handle and
type validation, through SteamVR, the driver and the client emulator, and skips the Micro QR, ArUco
and AprilTag tests as unsupported. Build `conformance_cli` from
[OpenXR-CTS](https://github.com/KhronosGroup/OpenXR-CTS) (`cmake --build build --target
conformance_cli`; keep the build path short on Windows) and, with a headset or the emulator
streaming, run from its output folder:

```sh
conformance_cli.exe "XR_EXT_spatial_marker_tracking-qr-code" -G D3D11 -H -T
conformance_cli.exe "XR_EXT_spatial_marker_tracking-qr-code-interactive" -G D3D11
```

The interactive test draws a box on every code it finds, magenta when the code's data was read;
complete it with the select button. Every marker test is tagged `[no_auto]`, so it only runs when
named.

## Registration

The driver registers the layer when SteamVR starts, for the current user:
`HKCU\Software\Khronos\OpenXR\1\ApiLayers\Implicit`, value = path of `alvr_openxr_layer.json`,
`0` = enabled. It removes registrations of other ALVR copies (their manifest has the same file name),
and **Headset → OpenXR marker tracking** in the dashboard writes `1` (disabled) instead.

Packaged (AppContainer) applications, such as UWP apps, read neither that key nor the streamer folder
by default, so the driver grants `ALL APPLICATION PACKAGES` read access to the key and to the
`openxr_layer` folder. The pipe belongs to the user running SteamVR and is open only to their
processes, including their AppContainers, which may only read from it. The layer only trusts a pipe
created by its own user.

Unregistering the driver from SteamVR (in the dashboard, or by another ALVR copy taking over) also
removes the layer's registration. Where that cannot happen, as when an MSIX package is removed, the
layer finds its streamer folder is no longer among SteamVR's drivers and stays out of the way: it
only provides the extensions the loader advertises, without any QR code, so that applications
enabling them still start, and leaves everything else to the runtime.

Setting `DISABLE_XR_APILAYER_ALVR_MARKER_TRACKING` (any value) turns the layer off for one process.

## Debugging

- `cargo run -p alvr_marker_ipc --example dump_markers -- --watch` prints what the driver sends:
  texts quoted, bytes in hexadecimal.
- `cargo run -p alvr_openxr_layer --example marker_probe --release -- 15` runs a minimal OpenXR
  application (D3D11, empty frames) on SteamVR and prints the codes it gets, relative to the head
  and to `STAGE`, and at the end how much each one jittered from frame to frame. The headset shows
  black while it runs.
- The layer writes to the debugger output (DebugView shows it), and to a file when
  `ALVR_OPENXR_LAYER_LOG` names one.
- `XR_LOADER_DEBUG=all` makes the OpenXR loader explain which layers it loads and why.
- `cargo test -p alvr_openxr_layer` drives the layer, as the loader and an application would, on a
  fake runtime.

## Limitations

- Windows only; the pipe has no Linux implementation yet.
- 64-bit applications only.
- QR codes only: the headset detects no other marker type.
- A runtime without `XR_KHR_win32_convert_performance_counter_time` gets the latest poses, whatever
  the requested time. SteamVR has it.
- No `xrEnumerateSpatialCapabilities` result until a headset has connected, so an application that
  starts SteamVR itself may need to query again once the headset is up.
- Without `XR_KHR_win32_convert_performance_counter_time` on the runtime, recentering is not
  reported. SteamVR has it.
- A code moved in one jump, which only the emulator does, is predicted past where it lands for
  about a frame, as if it kept moving.
- The layer provides both spatial extensions together: on a runtime with `XR_EXT_spatial_entity` but
  not `XR_EXT_spatial_marker_tracking`, the runtime's own spatial entities are hidden.
