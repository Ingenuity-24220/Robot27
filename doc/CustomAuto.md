# Custom Auto Runner and Tuners

A replacement for Pedro Pathing tuning. It drives to field poses with a PD + kS controller
on the Pinpoint pose, writing motor powers directly (no Drivetrain smoothing filter).

| File | What it is |
|---|---|
| `TeamCode/.../util/AutoConfig.java` | All settings: motor names/directions, Pinpoint pods/offsets, gains, tolerances |
| `TeamCode/.../HardwareAndPinpointTuner.java` | TeleOp **"1. Hardware & Pinpoint Tuner"** |
| `TeamCode/.../DrivePIDTuner.java` | TeleOp **"2. Drive PID Tuner"** |
| `TeamCode/.../CustomAutoRunner.java` | Autonomous **"Custom Auto Runner"** |

Run them in order: Tuner 1, then Tuner 2, then the runner. Values changed on the gamepad are
**not saved**: copy them into `AutoConfig.java` and redeploy.

Live values show in Panels (graph `x`/`targetX`, etc.) and on the Driver Station.

## Coordinates

Pedro's frame, so poses from the Pedro visualizer work as-is:

- Field is 0-144 in on both axes, `(0, 0)` is the bottom-left corner.
- +X right, +Y up. Heading 0° faces +X, CCW is positive.
- Mirror for the other alliance: `x → 144 − x`, `heading → 180 − heading`.

## Step 1: Hardware & Pinpoint Tuner

Put the robot on the field and keep it still for 1 s after INIT, then press START.

1. **Wheels.** Hold one button at a time. Each wheel must spin **forward**.
   - A = LF, B = RF, X = LR, Y = RR
   - If one spins backward, flip its `*_DIR` in `AutoConfig`.
2. **Pods.** Press D-pad Down to zero the pose, then push the robot by hand:
   - Forward → `X (in)` goes **up**.
   - Left → `Y (in)` goes **up**.
   - Turn CCW → `Heading (deg)` goes **up**.
   - If X goes the wrong way, flip `X_POD_DIR`. If Y does, flip `Y_POD_DIR`.
     If heading is wrong, the Pinpoint is mounted upside down or the pods are swapped.
3. **Pod offsets.**
   1. Press Left Bumper. Telemetry shows `ZERO_OFFSET` and the pose resets.
   2. Hold Right Trigger to spin the robot in place until heading is about 180° (doesn't need to be exact;
      anything over ~90° works, but stay away from 360°). Go slowly, and let it stop fully before step 3.
      Heading must go **up** while spinning; if it goes down, the turn direction is wrong.
   3. Press Right Bumper. Copy `Computed X_POD_OFFSET_MM` and `Computed Y_POD_OFFSET_MM`
      into `AutoConfig`. Repeat 2-3 times and average if they vary.
   4. Press Left Bumper again to go back to `CONFIG_OFFSET`.
   5. Check: spin the robot in place with the new offsets. X and Y should stay within about 1 in.

## Step 2: Drive PID Tuner

Wherever the robot is at START becomes (24, 24) facing 0°. Leave at least 2 ft of space
in front of and to the left of the robot.

| Button | Action |
|---|---|
| Left Bumper | RUNNING / PAUSED. **Starts PAUSED. Use it as the kill switch.** |
| A | Toggle 24 in forward / back |
| X | Toggle 24 in strafe left / back |
| B | Toggle 90° CCW turn / back |
| Y | Toggle diagonal (24, 24) + 45° / back |
| D-pad Up / Down | Select gain |
| D-pad Left / Right | Decrease / increase selected gain |
| Right Bumper | Step size: 0.1 → 0.01 → 0.001 |

Press Left Bumper to start, then:

1. **`kP_xy`** with A. Raise it until the robot gets there quickly and just starts to overshoot.
2. **`kD_xy`** with A. Raise it until the overshoot is gone. Too high makes the robot jittery.
3. **`kS_xy`** with A. If it stops about 1 in short, raise it (typically 0.08-0.12).
   If it chatters back and forth at the end, lower it.
4. Check strafing with X. If strafing undershoots a lot more than driving forward, that's
   mecanum slip; the shared gains are a compromise, so tune for both.
5. **`kP_h`, `kD_h`, `kS_h`** with B, the same way.
6. **Final check** with Y. The robot should drive diagonally and turn at the same time,
   ending at the target. If it spirals or drives away, a direction in Step 1 is wrong.

Copy the six gains from telemetry into `AutoConfig.java`.

Other knobs in `AutoConfig`: `POS_TOL_IN` (1.0 in) and `HEAD_TOL_DEG` (2°) are when a move counts
as "there". The kS term turns off inside these tolerances.

## Step 3: Custom Auto Runner

Set the start pose and write the routine in `CustomAutoRunner.java`:

```java
private final double startX = 59.521, startY = 34.411, startH = 180.0;

private void runPath() {
    driveTo(23.026, 34.411, 180, 0.6);
    followBezier(23.026, 34.411, 44.315, 47.793, 23.026, 65.816, 180, 180, 0.6);
    waitSec(0.25);
    turnTo(90, 0.5);
}
```

The included example is meowAuto's line + curve to park. Place the robot at the start pose,
INIT, wait 1 s, then START.

### API

All headings are in degrees, all distances in inches, `maxPower` is 0-1.

| Method | What it does |
|---|---|
| `driveTo(x, y, h, maxPower)` | Drive to the pose and stop. Done once within tolerance for 0.15 s. 3 s timeout. |
| `driveTo(x, y, h, maxPower, tolIn, tolDeg, timeoutSec)` | Same, with custom tolerances and timeout. |
| `driveThrough(x, y, h, maxPower[, switchRadiusIn])` | Pass through a waypoint without stopping. Moves on once within the radius (default 4 in). |
| `turnTo(h, maxPower)` | Turn in place, holding the current position. 2 s timeout. |
| `waitSec(seconds)` | Stop and wait. |
| `followBezier(x0, y0, cx, cy, x2, y2, startH, endH, maxPower)` | Quadratic curve from the visualizer (start, control, end). Sampled into 10 points: 9 `driveThrough`, then a final `driveTo`. Heading turns the short way from `startH` to `endH`. |
| `setStartPose(x, y, h)` | Reset the Pinpoint pose. |

Every move returns `false` on timeout or STOP, and all loops stop when STOP is pressed.

To run intake/outtake, call their methods between moves in `runPath()`.

## Troubleshooting

- **Robot drives away from the target or spins forever:** a wheel or pod direction is wrong. Redo Step 1.
- **X/Y drift while turning in place:** pod offsets are wrong. Redo Step 1.3.
- **Stops short of the target:** raise `kS_xy` / `kS_h`, or check the timeout isn't ending the move.
- **Oscillates at the end:** lower `kP`, raise `kD`, or lower `kS`.
- **Heading jumps at INIT:** the robot moved during the Pinpoint IMU calibration. Keep it still for 1 s after INIT.

## Data logs

Tuner 2 and the Custom Auto Runner each write one CSV per run to `/sdcard/FIRST/data` on the
Control Hub (`tuner_YYYYMMDD_HHMMSS.csv` / `auto_YYYYMMDD_HHMMSS.csv`). Telemetry shows the
file name as `Log file`. Tuner 2 only logs while RUNNING.

Pull them over USB or wireless ADB:

```bash
adb connect 192.168.43.1:5555
adb pull /sdcard/FIRST/data/ ./logs/
```

Columns: time (s), test segment / auto step, target and actual pose (in, deg), Pinpoint
velocities, robot-frame errors, filtered derivatives (in/s, deg/s), controller outputs before
power scaling, and the gains in use (so D-pad changes show up in the data).
