# Robot27

This is our robot code for 2027.

This main branch is built on the official [FTC Robot Controller SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController) **v12.0**.
To pull in a future SDK release:

```
git remote add upstream https://github.com/FIRST-Tech-Challenge/FtcRobotController.git   # first time only
git fetch upstream
git merge upstream/master
```

The SDK's own release notes are at https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases.

## Outtake tuning

Step-by-step controls for the **Outtake Tuner** OpMode are in [doc/CustomAuto.md](doc/CustomAuto.md#outtake-tuner).
This section covers what to tune and how to check it.

**Tune the PIDF once per flywheel, not once per power.** kF ("power needed per unit of speed")
is about the same at every speed, and kP only corrects what kF misses, mostly the speed dip after
a shot. What changes with shooting position is the **target speed** (ticks/s), not the gains.

1. **Tune gains at the highest speed you use.** Set `NECTAR_POWER` / `POLLEN_POWER` in
   `util/Outtake.java` to that power (for example 0.9), press B to measure kF, then tune kP. A shot
   hurts most at high speed.
2. **Check one lower speed.** Lower the target (D-pad Down) to about the 0.62 speed and set kP near 0.
   If the speed levels off within about 5% of the target, kF holds across the range. If not,
   friction matters at low speed and kF needs adjusting there.
3. **Find a speed for each shooting position.** At each spot, adjust the target until shots land
   and record the ticks/s. Pressing B at a power shows the speed that power gives, which is a good
   starting point.

| Shooting position | Nectar (ticks/s) | Pollen (ticks/s) |
|---|---|---|
| _fill in_ | | |

### Checking that the PIDF isn't hurting consistency

From the same spot, fire about 10 shots with fixed power (`start(type, 0.9)`) and about 10 with
speed control (`startAtSpeed(type)`, opening the gate only once `atSpeed(type)` is true), and
compare where they land. Repeat on a fresh and a tired battery: fixed power drifts as the battery
drops; speed control shouldn't.

| Sign in the tuner telemetry / `outtake_*.csv` log | Meaning |
|---|---|
| Speed when the gate opens varies by more than ±40 ticks/s | Firing before it's ready: wait on `atSpeed()` |
| Speed overshoots, then rings after a shot | kP too high |
| Speed recovers slowly between shots | kP too low, or shots too close together |
| Speed is steady but shots still scatter | Not the flywheel: check ball feed, ball wear, robot position |

Note: `startAtSpeed` currently uses one target per flywheel (`NECTAR_VELOCITY` / `POLLEN_VELOCITY`).
Shooting from several positions needs a version that takes the speed as an argument.
