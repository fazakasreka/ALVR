# ALVR OpenXR marker layer

An implicit OpenXR API layer that gives **any** OpenXR application running on SteamVR + ALVR the QR
codes the headset sees, through the standard `XR_EXT_spatial_entity` and
`XR_EXT_spatial_marker_tracking` extensions.

SteamVR's OpenXR runtime implements neither extension, and an OpenVR driver cannot add OpenXR
extensions. So the layer, loaded by the OpenXR loader into each application, implements them on top
of the runtime:

```text
headset ── QR codes (ClientControlPacket::Markers) ──▶ ALVR driver in vrserver.exe
                                                            │ named pipe (alvr_marker_ipc)
app ─▶ OpenXR loader ─▶ this layer ─▶ SteamVR OpenXR runtime
```

- **The driver owns the markers.** It recenters them like the headset pose and serves them on the
  named pipe `\\.\pipe\ALVR_Markers` (see `alvr/marker_ipc`): each connected layer gets the
  complete current state when it connects and whenever it changes. Poses are in the space the
  driver submits the headset pose in, which SteamVR's OpenXR runtime calls `STAGE` (the driver keeps
  the standing zero pose at identity).
- **The layer is thin.** It reads whatever has arrived when the application asks for markers, without
  waiting, and maps the latest state onto the extensions, locating its own `STAGE` space in the
  application's base space. A broken pipe means the driver is gone.

## What is implemented

| | |
|---|---|
| Capabilities | `XR_SPATIAL_CAPABILITY_MARKER_TRACKING_QR_CODE_EXT`, while the ALVR driver runs |
| Components | `MARKER` (QR text as a string buffer), `BOUNDED_2D` (centre pose and size) |
| Features | fixed size markers (`XrSpatialMarkerSizeEXT` overrides the detected size), static markers |
| Snapshots | discovery and update snapshots, `XrSpatialFilterTrackingStateEXT` |
| Entities | ids derived from the QR text, so stable across snapshots, contexts and SteamVR restarts |
| Tracking states | `TRACKING` while the headset sees the code, `PAUSED` otherwise or while disconnected |
| Events | `XrEventDataSpatialDiscoveryRecommendedEXT` when a context is created and when the set of codes changes |
| `XR_EXT_future` | provided when the runtime lacks it; futures of the layer are ready immediately |

ArUco, AprilTag and Micro QR are reported as unsupported: the headset only reports QR codes.

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
- Applications cannot ask for marker types or dictionaries: there is no channel from the layer back
  to the headset yet, and the headset only detects QR codes.
- Marker timestamps are not carried: poses are the latest known, which suits static markers.
- No `xrEnumerateSpatialCapabilities` result until the driver has started, so an application that
  starts SteamVR itself may need to query again once the driver is up.
