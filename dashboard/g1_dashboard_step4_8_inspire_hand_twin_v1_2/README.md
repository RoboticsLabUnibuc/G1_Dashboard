# Step 4.8 — Live Inspire RH56DFX hand twin (read-only)

This revision extends the working G1 body twin with the actual Inspire RH56DFX hand URDF/STL assets supplied from the robot's installed `xr_teleoperate_g1demo/assets/inspire_hand` directory. The browser now animates both physical hand feedback and the published finger command. Controller V1.8 is unchanged.

## What changes

- Replaces the generic fixed `left_rubber_hand` / `right_rubber_hand` visuals with articulated Inspire RH56DFX meshes when the hand assets load successfully.
- Solid Inspire hands use `hands.feedback_state` (right motors 0..5, left motors 6..11) as physical feedback.
- DFX motor order is preserved exactly as `[pinky, ring, middle, index, thumb bend, thumb rotation]`; the Live hand matrix uses the same order.
- Cyan transparent Inspire hands use `hands.current_left` / `hands.current_right` and are controlled by the existing **Command ghost** toggle.
- The fast `/api/pose` payload now carries the three hand arrays as well as 29-DoF body pose and 14 arm commands, so fingers update on the same latest-only ~30 Hz browser path instead of the slower status DOM path.
- Finger mimic joints are evaluated from the supplied URDFs: index/middle/ring/pinky intermediate joints mirror their proximal joint; thumb intermediate/distal use the URDF's 1.6x / 2.4x pitch mimic relationships.
- The controller's normalized Inspire convention is inverted back to URDF joint angles: the four fingers map `1=open -> 0 rad`, `0=closed -> 1.7 rad`; thumb pitch maps to 0..0.5 rad; thumb yaw maps from normalized command back to -0.1..1.3 rad.
- If Inspire assets are missing, the renderer falls back to the original generic rubber-hand body meshes.

## G1 wrist mounting

The supplied standalone Inspire URDFs do not contain the G1 wrist attachment. The renderer uses the G1 rev_1_0 + RH56DFX mounting convention matching the same G1 wrist frames: left hand root rotates +90° about Z; right hand root rotates 180° about X and -90° about Z; both mount at the `*_wrist_yaw_link` origin. These transforms are kept as explicit constants in `static/inspire_hand_model.js` so they can be adjusted if a physical adapter differs.

## Assets

Bundled Inspire files:

```text
static/model/inspire_hand/
├── meshes/                 # 26 supplied STL files
└── urdf/
    ├── inspire_hand_left.urdf
    ├── inspire_hand_right.urdf
    └── inspire_hand.yml
```

The 36 Unitree G1 body meshes are still not duplicated in the ZIP. Copy them from the previous dashboard or run `fetch_g1_assets.py`. Verification now checks both sets:

```bash
python3 fetch_g1_assets.py --verify-only
```

Expected after copying the existing G1 body meshes:

```text
G1 mesh assets: 36/36 present
Inspire RH56DFX mesh assets: 26/26 present
```

## Safety boundary

This remains read-only. The bridge still has no POST/control endpoint and no DDS import. Adding hand arrays to `/api/pose` only changes visualization telemetry. Browser disconnects still have no robot-control effect.

---

# Step 4.7.1 — Controller action readiness + hand panel polish (read-only)

This revision adds a controller-owned `actions` telemetry object and displays the exact next `c`-equivalent transition as `READY` or `BLOCKED`. **There is still no browser command endpoint.** Controller V1.8 computes readiness from the same state-machine policy that the future request handler will reuse.

The working Step 4.6 joint-health overlay, system monitor, RobotState service inventory, IMU/odometry, camera, and twin remain unchanged.

# G1 Dashboard Step 4.6 v1.1 — Joint Health Overlay + Twin Controls Polish


## Step 4.7.1 UI fix

The Live Hands card was redesigned for the narrow operator rail. It now uses one six-row matrix with LEFT and RIGHT side-by-side, rather than two compressed mini tables. The blue fill is the current command and the white vertical marker is Inspire state feedback (`feedback_state`). Numeric cells show command / feedback. Tracking and feedback freshness remain read-only telemetry.


## v1.1 UI polish

- Moves the mesh-twin display controls onto a dedicated control row so the model title/status no longer competes with checkboxes.
- Renames `Joints` to `Joint markers`.
- Joint markers are now intentionally drawn on top of the G1 mesh and are larger, so toggling them is visually obvious.
- Joint markers default OFF for a clean operational view; the currently selected joint marker remains visible.
- No telemetry, system-monitor, DDS, service, camera, or robot-control behavior changed.

This revision keeps the validated Step 4.5 v1.2 system/service/IMU/odometry stack and adds a read-only diagnostic overlay to the measured G1 mesh.

- Health overlay modes: Composite, Temperature, Torque.
- Temperature display bands: <70°C nominal, 70–79°C watch, 80–89°C high, >=90°C very high. These are dashboard visualization bands only, not Unitree safety thresholds.
- Torque utilization is `abs(tau_est) / URDF effort` using the official `g1_29dof_rev_1_0` URDF effort attributes. Display bands: <50% nominal, 50–74% watch, 75–89% high, >=90% very high. URDF effort values are references for visualization, not replacements for controller or Unitree limits.
- Selected Joint now shows torque utilization, URDF effort reference, health chip, and compact bars.
- No controller changes, DDS publishers, service switching, or browser command path are added.

# G1 Dashboard Step 4.5 v1.2 — base sensing + DDS startup fix

This revision keeps the validated V1.7 XR controller/control path unchanged and extends only the read-only dashboard side.

## Changes

- Inspect layout reorganized into:
  - PC2 system health
  - live torso IMU + best-effort odometry
  - runtime endpoints/processes
  - full-width Unitree RobotState service inventory
- Service inventory is larger, scrollable, has a sticky header, larger text, summary counts, and a name filter.
- `g1_dashboard_system_monitor.py` now initializes Unitree DDS once and reads:
  - RobotState `ServiceList()` + API versions only
  - `rt/secondary_imu` as `unitree_hg.msg.dds_.IMUState_`
  - best-effort `rt/odommodestate` and `rt/lf/odommodestate` as `unitree_go.msg.dds_.SportModeState_`
- No DDS publishers are created by the monitor.
- `ServiceSwitch()` is still never called.
- `start_dashboard.sh` starts both the read-only system monitor and HTTP dashboard bridge, so the dashboard side now needs one command.

## Safety boundary

The dashboard remains read-only:

- browser has no robot DDS access;
- bridge imports no Unitree SDK and has no command endpoints;
- system monitor only reads host status, service inventory, IMU, and odometry;
- the validated V1.7 teleoperation controller remains a separate process and is not modified by this package.

## Install

Copy/unzip this package on PC2, then copy the 36 G1 STL files from the previous working dashboard into:

```text
static/model/g1/meshes/
```

Verify them with:

```bash
python3 fetch_g1_assets.py --verify-only
```

Expected:

```text
G1 mesh assets: 36/36 present
```

## One-command dashboard start

```bash
cd "$HOME/xr_teleoperate_g1demo/dashboard/step4_6/g1_dashboard_step4_6_joint_health_v1"
./start_dashboard.sh
```

Defaults:

```text
interface:        enP8p1s0
RobotState:       auto / read-only
base sensing:     auto / read-only
system telemetry: 5 Hz
HTTP:             :8080
```

`start_dashboard.sh` automatically prefers `$HOME/miniconda3/envs/g1_xr/bin/python` for the Unitree monitor when present, while the bridge remains stdlib-only. `Ctrl+C` stops both dashboard-side processes.

Optional overrides:

```bash
G1_DASHBOARD_INTERFACE=enP8p1s0 \
G1_DASHBOARD_UNITREE_SERVICES=auto \
G1_DASHBOARD_BASE_SENSING=auto \
G1_DASHBOARD_SYSTEM_HZ=5 \
./start_dashboard.sh
```

## Base sensing

### Torso IMU

The monitor subscribes read-only to:

```text
rt/secondary_imu
```

and displays roll/pitch/yaw, quaternion, gyroscope, accelerometer, temperature, sample age, and sample count.

### Odometry

The monitor attempts read-only subscriptions to:

```text
rt/odommodestate
rt/lf/odommodestate
```

using the installed SDK's `SportModeState_` high-level state type. The panel shows decoded position, velocity, and yaw rate when packets are available. If the installed firmware/service uses a different type or the odometer service is off, the panel remains in a waiting/unavailable state without affecting anything else.

## Unitree service inventory

The monitor only calls `ServiceList()` and API-version reads. The inventory intentionally has no switches/buttons in Step 4.5.

Future service switching must go through the same controller-mediated action-request architecture planned for dashboard controls. In particular, the browser must never call `ServiceSwitch()` directly. Protected services should remain non-actionable, and any non-protected service actions should be explicitly whitelisted and acknowledged by the controller/action broker.


## v1.1 service-state correction

Unitree RobotState service status uses vendor polarity `0 = ON`, `1 = OFF`. Step 4.5 v1 displayed that polarity backwards. v1.1 preserves the raw `status` field for diagnostics and additionally publishes a normalized `enabled` boolean. The UI uses `enabled` (with a raw-status fallback) so service inventory state now matches the robot. This revision remains read-only and still never calls `ServiceSwitch()`.


## v1.2 DDS startup serialization fix

PC2 exposed a startup race between `RobotStateClient.Init()` and creation of the
complex `SportModeState_` odometry Topic. The resulting CycloneDDS TypeObject
encoding exception could also make the simultaneously-initializing RobotState
client fail with `DDS_RETCODE_BAD_PARAMETER`.

This revision constructs and validates the read-only RobotState RPC client
synchronously first, then constructs the secondary IMU subscriber. Odometry runs
in a spawned read-only helper with its own Unitree ChannelFactory/DomainParticipant.
That isolates the complex `SportModeState_` TypeObject from RobotState and IMU.

If the installed Python/CycloneDDS combination still cannot construct
`SportModeState_`, only odometry is marked unavailable with a short UI diagnostic;
service inventory and secondary IMU remain available. The full exception is printed
only in the monitor terminal. No dependency versions are changed and the dashboard
still starts with the same single `./start_dashboard.sh` command.

## Step 4.7.2 Live rail layout fix

The Live operator rail now preserves the minimum height of health, quick-status, action-readiness, hand, and selected-joint panels instead of allowing CSS Grid to compress their contents below their usable size. On shorter browser windows the rail becomes a narrow internal vertical scroller rather than clipping hand rows or allowing status text to bleed into the next card. No telemetry or robot-control behavior changed.


## Step 4.8.1 — Inspire feedback validity guard

Unitree documents `rt/inspire/state` `q` in normalized `[0,1]` units. Some dual-RH56DFX installations return stale/out-of-range values from the DFX service. The dashboard now validates all six channels per hand before treating them as measured state. If a side is invalid, the solid hand is **command-estimated for visualization only** and the cyan finger ghost is hidden for that side to avoid implying measured feedback. The Hands panel shows `INVALID RANGE` and suppresses misleading feedback markers. This has no effect on robot commands, controller safety, or DDS.


## Step 4.8.2 — Restore Inspire command ghost with invalid feedback

When RH56DFX feedback is invalid, the solid articulated hand still uses the current
command as a visualization-only fallback. Unlike Step 4.8.1, the cyan command hand
is no longer hidden. It is rendered with a slightly stronger transparent material
and polygon offset so it remains visible as a cyan shell even when it is exactly
coincident with the command-estimated solid hand. If valid feedback returns, the
solid hand automatically becomes measured again and the cyan ghost continues to
represent the command independently. No controller, DDS, command, or safety logic
changes are included.
