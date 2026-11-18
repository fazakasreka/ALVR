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
  supports the options, plus one for marker recentering.
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
| Components | `MARKER` (QR text as a string buffer), `BOUNDED_2D` (centre pose and size) |
| Features | fixed size markers, static markers; both always honoured, see below |
| Snapshots | discovery and update snapshots, `XrSpatialFilterTrackingStateEXT` |
| Poses | at the requested time: interpolated between the headset's poses, predicted up to 100 ms past the latest |
| Entities | ids derived from the QR text, so stable across snapshots, contexts and SteamVR restarts |
| Tracking states | `TRACKING` while the headset sees the code; `PAUSED` otherwise, while disconnected, or when nothing came for 500 ms |
| Events | `XrEventDataSpatialDiscoveryRecommendedEXT` when a context is created and when the set of codes changes |
| `XR_EXT_future` | provided when the runtime lacks it; futures of the layer are ready immediately |

ArUco, AprilTag and Micro QR are reported as unsupported: the headset only reports QR codes.

**Marker options.** A context's `XrSpatialMarkerSizeEXT` and `XrSpatialMarkerStaticOptimizationEXT`
go to the headset, which uses them for more precise poses when it supports them. The layer honours
them either way, as the extension requires: a context with a size gets every code at that size, and
a context for static codes gets their poses averaged over the last second instead of predicted.

**Coexistence.** The layer first creates the instance with the extensions the application asked
for. If the runtime accepts them, it implements them itself and the layer steps aside. An
application that enables none of the extensions talks to the runtime directly: the layer only sits
in `xrGetInstanceProcAddr` and `xrDestroyInstance`.

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

Setting `DISABLE_XR_APILAYER_ALVR_MARKER_TRACKING` (any value) turns the layer off for one process.

## Debugging

- `cargo run -p alvr_marker_ipc --example dump_markers -- --watch` prints what the driver sends.
- `cargo run -p alvr_openxr_layer --example marker_probe --release -- 15` runs a minimal OpenXR
  application (D3D11, empty frames) on SteamVR and prints the codes it gets, relative to the head
  and to `STAGE`. The headset shows black while it runs.
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
- Reference space changes (`XrEventDataReferenceSpaceChangePending`) are not reported when ALVR
  recenters.
