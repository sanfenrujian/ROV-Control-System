# Motion Control Optimization Notes

Date: 2026-05-15

## Scope

This update focuses on the v2 control path in `motion_control_v2.c` and `thread_manager_v2.c`.

The main fixes are:

1. `ch4` yaw input is now part of the horizontal thruster mix.
2. `M4` is no longer idle during heave/depth control.
3. Depth zero-offset is applied to `g_sys.depth.true_depth`.
4. Heading-hold is wired into the runtime mode state machine.
5. Legacy JY901B / telemetry structs are bridged back into `g_sys` for compatibility.

## Remote Channel Mapping

The optimized v2 path uses this mapping:

- `ch1`: surge, forward/backward
- `ch2`: sway, left/right translation
- `ch3`: heave or depth target trim
- `ch4`: yaw command / heading trim
- `ch7`: OTA trigger
- `ch8`: arm motors
- `ch9`: depth hold enable
- `ch10`: heading hold enable

All analog channels are expected in the normalized `0.0 ~ 2.0` range with `1.0` as center.

## Mode Logic

Mode priority is now:

1. `MODE_FAILSAFE` when motors are not armed
2. `MODE_DEPTH_HOLD` when `ch9 > 1.5`
3. `MODE_HEADING_HOLD` when `ch10 > 1.5`
4. `MODE_MANUAL` otherwise

While in `MODE_DEPTH_HOLD`, `ch10` can still enable heading stabilization overlay.

## Control Changes

### Manual

- `ch4` directly contributes to differential yaw thrust on `M1/M2`
- vertical thrust is split across `M3/M4`

### Depth Hold

- `ch1/ch2` remain manual for horizontal movement
- `ch3` trims the depth target instead of being ignored
- PID output is distributed to both vertical thrusters

### Heading Hold

- target heading is captured when the mode is entered
- `ch4` slews the heading target instead of disabling the mode
- if a valid absolute yaw becomes available later, the controller will use it automatically

## Heading Source Fallback

The current v2 project still does not show a confirmed JY901B receive/parser chain.

Because of that, heading-hold now uses this priority:

1. `g_sys.jy901b.yaw` when legacy JY901B data is available
2. `g_sys.imu.yaw` when a real absolute yaw is available
3. integrated `gyro_z` estimate as a fallback

The gyro-only fallback is useful for short-term stabilization, but it will drift over time. If long-duration heading lock is required, the JY901B data path should be fully restored and verified on hardware.

## Compatibility Bridge

To avoid breaking old telemetry/data packaging code, the runtime now mirrors these values back into the legacy structures:

- `send_angle`
- `true_depth`
- `ms5837`
- `PS_2`
- `jy901b`-derived attitude into `g_sys`

## Recommended Hardware Validation

1. Arm with `ch8` and confirm all four ESC outputs stay centered at idle.
2. Move `ch4` in manual mode and confirm `M1/M2` produce opposite yaw torque.
3. Move `ch3` in manual mode and confirm both `M3/M4` respond together.
4. Enter depth hold with `ch9` and verify `true_depth` is near `0` at the calibrated surface point.
5. Enter heading hold with `ch10` and verify short-term yaw stabilization.
6. If heading drifts badly, inspect the missing JY901B receive path before increasing PID gains.
