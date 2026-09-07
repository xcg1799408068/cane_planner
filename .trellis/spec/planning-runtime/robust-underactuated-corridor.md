## Robust underactuated corridor contract

### 1. Scope / Trigger
This contract applies when `dwc/robust_speed_interval_enable` is enabled. It covers the cross-layer flow from launch parameters and odometry to `DynamicWalkingCorridor`, MPPI actionability, diagnostics, and RViz markers.

### 2. Signatures
- `DynamicWalkingCorridor::Config`: `robust_speed_interval_enable`, `robust_speed_interval_mode` (`fixed|observed`), `user_speed_min`, `user_speed_max`, `user_speed_floor`, `observed_speed_margin`, `observed_speed_window`, and `observed_speed_interval_valid`.
- `DynamicWalkingCorridor::Result`: `robust_requested`, `robust_active`, `robust_config_valid`, and `robust_reject_reason`.
- `MpcController::setRobustSpeedInterval(bool enabled, double speed_min, double speed_max, double speed_floor)` must receive the resolved interval used to build the timed corridor before `plan()`.

### 3. Contracts
- Fixed mode uses the launch interval unchanged and is the reproducible experiment mode.
- Observed mode derives a bounded interval from recent odometry planar speed samples: `min=max(v_floor, observed_min-margin)` and `max=observed_max+margin`; no samples in the configured window means the interval is unavailable.
- Valid robust activation requires finite values and `0 < v_floor <= v_min <= v_max`, a supported mode, and an available observed interval when mode is `observed`.
- A robust segment preserves nominal `t_start/t_end`, and adds `[t_lower,t_upper] = [s_start/v_max, s_end/v_min]`. Queries use all segments whose robust windows overlap the query time.
- Dynamic occupancy over `[t_lower,t_upper]` is a conservative swept union under the linear pedestrian model. Each sample footprint is expanded by `|v_ped| * sample_dt`; the final endpoint footprint is included.
- MPPI actionability replays one steering sequence from one initial state at slow/nominal/fast speeds. Every rollout checks CoM corridor geometry and dynamic occupancy, static collision, dynamic clearance, and FOV-gated LFPC foot placement. The resolved interval is used for both corridor timing and actionability.
- RViz uses stable robust namespaces: `mpc_walking_corridor_robust_polygons` and `mpc_walking_corridor_robust_occupancy`; debug text includes mode, bounds, active/invalid status, and reject reason.

### 4. Validation & Error Matrix
| Condition | Result |
|---|---|
| Robust disabled | Nominal timing/query behavior; no robust actionability |
| Valid fixed interval | `robust_active=true`, interval occupancy and three-speed checks enabled |
| Valid observed samples | Resolved interval is used by DWC and MPPI |
| Observed window empty | `robust_active=false`, reason `OBSERVED_SPEED_UNAVAILABLE` |
| Unsupported mode | `robust_active=false`, reason `INVALID_SPEED_MODE` |
| Nonfinite/reversed/below-floor interval | `robust_active=false`, reason `INVALID_SPEED_INTERVAL` |
| No actionable three-speed sequence | Candidate cost is infinite and reject count increments |

### 5. Good/Base/Bad Cases
- Good: fixed `[0.7,1.3]`, floor `0.05`; timed segments expose wider arrival windows and all three same-steering rollouts pass.
- Base: robust disabled; existing nominal corridor fields and tests remain unchanged.
- Bad: robust requested with `v_min < v_floor`, or observed mode with no samples; do not silently advertise robust activation or use a mismatched MPPI interval.

### 6. Tests Required
- Assert fast-start/slow-end arrival bounds and overlapping robust segment lookup.
- Assert coarse-step swept occupancy detects a pedestrian between stored sample times.
- Assert invalid fixed and unavailable observed intervals expose explicit result reasons.
- Assert nominal mode preserves `t_lower/t_upper` compatibility.
- Assert launch/build and focused path-searching tests pass; runtime smoke should verify robust launch starts without ROS errors.
- For controller regressions, assert foot-placement collision and corridor geometry can reject a shared steering sequence at any representative speed.

### 7. Wrong vs Correct
#### Wrong
```cpp
// DWC observes [0.45, 0.75], while MPPI silently keeps launch [0.7, 1.3].
updateCorridorFromOdom();
mpc_controller_->plan(...);
```

#### Correct
```cpp
const auto cfg = dynamic_walking_corridor_->getConfig();
mpc_controller_->setRobustSpeedInterval(
    corridor_result.robust_active && cfg.robust_speed_interval_enable,
    cfg.user_speed_min, cfg.user_speed_max, cfg.user_speed_floor);
mpc_controller_->plan(...);
```

The explicit status and shared resolved interval prevent a nominal/robust mismatch from being mistaken for a safety guarantee.
