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
- In the team path (routine 6), every heading is the direction the **outtake** faces
  (`AutoConfig.AUTO_FRONT_OFFSET_DEG = 180`). Set it to 0 to use the intake side instead.
  Test routines 1-5, the tuners, and TeleOp use the intake side as the front.
- When AUTO ends (or STOP is pressed), the drive and outtake motors stop and both gates close.

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

During INIT, D-pad Up/Down sets the speed (0.2-1.0) and D-pad Left/Right picks the routine:
the example (meowAuto's line + curve, a turn, back to start), two laps around the center
structure (strafing, or facing the travel direction), an S-curve with turning while driving,
and the example three times in a row. Every routine ends back at the start pose, so tape the
start position and compare. Place the robot at the start pose, INIT, wait 1 s, then START.

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

### Team path start (routine 6)

Put the robot against the bottom (audience, y = 0) wall with the **intake end touching the wall**
and the outtake facing 90° (toward +Y), centered at x = 57.9. Start Y is
`AutoConfig.WALL_TO_CENTER_IN` (default 9.0 = half an 18 in robot): measure the distance from the
robot's center to the intake-side edge and set it, or every target will be off by the difference.

## Outtake Tuner

`Outtake.start(type, power)` runs a flywheel at a fixed power, as before (TeleOp 0.62 / 0.725,
AUTO 0.8 / 0.9). A fixed power gives a different speed as the battery drains and recovers slowly
after a shot. `Outtake.startAtSpeed(type)` instead holds a tuned **speed** (ticks/s) with the motor
encoder. Nothing uses `startAtSpeed` yet: once a flywheel is tuned, swap a `start(..., power)` call
for `startAtSpeed(...)` where you want consistent shots. Until tuned (`*_VELOCITY = 0` in
`util/Outtake.java`), `startAtSpeed` falls back to `NECTAR_POWER` / `POLLEN_POWER`.

Needs: an encoder cable from each flywheel motor to its motor port. Use a charged battery.

| Button | Action |
|---|---|
| X | Switch flywheel (nectar = `outtakeMotor`, pollen = `outtakeMini`) |
| B | Measure kF (runs 3 s at `NECTAR_POWER` / `POLLEN_POWER`) |
| Y | Spin up / stop at the target speed |
| D-pad Up/Down | Target speed ±20 ticks/s |
| LB + D-pad Up/Down | kF ±1% |
| D-pad Right/Left | kP ×1.25 / ÷1.25 |
| RB (hold) | Open the gate to fire |

In Panels, graph `target` and `velocity`. For each flywheel:

1. **Measure kF.** Press **B**. It sets kF, sets the target to the speed the old power gave
   (so shots should match that power; set `*_POWER` to the power you shoot at, e.g. 0.9 for
   AUTO, before measuring), and sets kP = 0.1 × kF.
   - "No speed reading": the encoder cable is missing.
   - "Velocity is negative": the encoder counts backward; flip the motor's direction in `Outtake`.
2. **Check F alone.** Set kP to near 0 (D-pad Left several times), press **Y**. The speed should
   level off a little below the target. If it's well above or below, adjust kF with LB + D-pad.
3. **Raise kP** (D-pad Right) until the speed reaches the target quickly. Watch:
   - **Overshoot** above ~40 ticks/s, or the line oscillates: kP too high, press D-pad Left.
   - **Rise time** slow, or it sits below the target: kP too low.
4. **Shot test.** With it at speed, feed a ball and hold **RB**. Look at **Shot dip** and
   **Shot recovery**. Raise kP until recovery is fast without overshoot afterward. Recovery time
   is how long to wait between shots.
5. **Find the right speed.** Change the target with D-pad Up/Down until shots land where you want.
6. **Copy the three lines** under "Copy into Outtake.java" (`*_VELOCITY`, `*_KP`, `*_KF`) into
   `util/Outtake.java`, then do the other flywheel (X).

`Outtake.atSpeed("nectar")` returns true when that flywheel is within `READY_TOLERANCE` (40
ticks/s); you can wait on it before opening a gate.

**Volleys (TeleOp `meow`):** LB fires a nectar volley and RB a pollen volley (`Outtake.fire(type)`;
the OpMode must call `outtake.update()` every loop). Press again to cancel. Each volley is a pattern
of gate pulses: `NECTAR_PATTERN` and `POLLEN_PATTERN` {1, 1, 0} in `util/Outtake.java` = one ball,
one ball, then the gate held open for `HOLD_OPEN_SEC` (1.0 s) to let the last balls through. Pulse
lengths: nectar 0.11 s for 1 ball; pollen 0.30 s for 1 ball. The next pulse
waits at least `MIN_GAP_SEC` 0.6 s, then fires once `atSpeed()`, or after `MAX_GAP_SEC` 1.0 s
regardless. In `meow`, D-pad Up/Down picks the speed preset (close / medium / far: `NECTAR_PRESETS` {1768, 2100, 2380}, `POLLEN_PRESETS` {1676, 1840, 2070} in `util/Outtake.java`; medium and far start from AUTO's 0.8 / 0.9 power and need tuning at their spots). During INIT, X switches the bumpers between volleys and "gate open while held".
In Custom Auto Runner, X during INIT switches the team path's shooting between volleys at the
tuned speed and the old fixed power (0.8 / 0.9) with the gates held open. Logs go to `outtake_<time>.csv`.

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

## Heading source (Pinpoint IMU vs hub IMU)

`AutoConfig.USE_HUB_HEADING` picks where heading comes from (see `util/Localization.java`):

- `true` (current default): the Pinpoint's IMU is ignored. X/Y are computed in our code from
  the Pinpoint's two raw pod counts, using the Control Hub IMU for heading. Use this while the
  Pinpoint's heading is unreliable (for example if the board can rock on its mount).
- `false`: the Pinpoint's own pose is used.

Telemetry shows `Heading source`, both IMUs' headings, and their difference; the logs record both.

When switching to `true`, check in Tuner 1:

1. `Raw Encoders` X goes up when pushing the robot forward and Y goes up when pushing it left.
   If not, set `X_ENC_SIGN` / `Y_ENC_SIGN` to `-1`. Then X/Y should behave like the push test above.
2. Hub orientation: turning CCW makes `Hub IMU heading` go up (`HUB_LOGO` / `HUB_USB`).
3. Redo the pod offsets (Step 1.3) and the 180° check, since earlier offsets were measured
   with the Pinpoint's heading.
