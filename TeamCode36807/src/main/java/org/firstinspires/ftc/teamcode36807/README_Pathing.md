# FTC Mecanum Path Chaining

**Setup, goBILDA Pinpoint odometry, testing, and PD controller tuning**

- **Platform:** FIRST Tech Challenge (FTC), SDK 12.0
- **Language:** Java
- **Drivetrain:** Mecanum
- **Localization:** goBILDA Pinpoint
- **Controller:** `MecanumPathFollower`
- **Test OpMode:** `PathChainingTeleOP`

> **Important:** Configure and verify odometry **before** tuning the path follower. An incorrect Pinpoint offset can create a false position error that makes a correctly functioning controller move sideways.

## Contents

1. [System overview](#system-overview)
2. [Path chaining test](#path-chaining-test)
3. [Pinpoint configuration](#pinpoint-configuration)
4. [Coordinate systems and alignment](#coordinate-systems-and-alignment)
5. [Odometry verification](#odometry-verification)
6. [Understanding PD control](#understanding-pd-control)
7. [Path follower parameters](#path-follower-parameters)
8. [Tuning procedure](#tuning-procedure)
9. [Lookahead and path recovery](#lookahead-and-path-recovery)
10. [Station keeping](#station-keeping)
11. [Mecanum drivetrain configuration](#mecanum-drivetrain-configuration)
12. [Planner and coordinate system](#planner-and-coordinate-system)
13. [Loop timing and rotation segments](#loop-timing-and-rotation-segments)
14. [Troubleshooting](#troubleshooting)
15. [New robot checklist](#new-robot-checklist)
16. [Lessons learned](#lessons-learned)
## System overview

The path-chaining system continuously uses the robot's estimated position and heading to command the mecanum wheels. It includes Bézier path planning, lookahead targeting, translation PD correction, proportional heading correction, acceleration/deceleration limits, recovery from path deviations, rotation-only segments, and final station keeping.

| Component | Responsibility |
| --- | --- |
| `PathChainingTeleOP` | Builds and executes the demonstration route |
| `PathSequence` | Defines waypoints and desired headings |
| `PathPlanner` | Plans the route |
| `PlannedRoute` | Stores route segments |
| `PathSegment` | Represents one segment |
| `BezierPath` | Defines curved path geometry |
| `MecanumPathFollower` | Computes wheel commands from pose and targets |
| `MecanumDrive` | Wraps drivetrain motors |
| `GoBildaPinpointDriver` | Reports position and heading |
| `BeezyBot` | Original robot-specific hardware configuration |

For another robot, adapt its hardware class, motor directions, Pinpoint settings, dimensions, and controller gains. The planning classes need not change merely because the chassis changes.

## Path chaining test

The test OpMode executes:

1. Drive **24 inches forward**.
2. Rotate **180°** approximately in place.
3. Return to the starting position while facing the reverse heading.
4. Enable final position correction (station keeping).

With an initial heading of 0° and the planner origin offset:

| Stage | X (in) | Y (in) | Heading |
| --- | ---: | ---: | ---: |
| Start | 72 | 72 | 0° |
| Forward | 96 | 72 | 0° |
| Rotate | 96 | 72 | 180° |
| Return | 72 | 72 | 180° |

The OpMode converts raw Pinpoint coordinates into planner coordinates:

```java
Pose2D plannerPose = new Pose2D(
    pose.x + 72.0,
    pose.y + 72.0,
    pose.heading
);
```

This is a **coordinate offset**, not a physical field localization procedure. Resetting Pinpoint does not automatically establish the robot's true position on the field.

## Pinpoint configuration

### Tracking wheel placement and offset signs

Measure both odometry pod offsets from the **same tracking point** (normally the robot center). Per the driver's `setOffsets()` documentation:

| Pod | Positive offset | Negative offset |
| --- | --- | --- |
| X pod (forward) | Left of center | Right of center |
| Y pod (strafe) | Forward of center | Behind center |

On the original robot, the X pod was 90 mm to the **right** and the Y pod was 182.88 mm **behind** the center:

```java
odom.setOffsets(-90, -182.88, DistanceUnit.MM);
```

**Do not reuse these measurements on a different robot.** Measure its pod locations and apply the documented signs.

### Encoder directions

The original robot used:

```java
odom.setEncoderDirections(
    GoBildaPinpointDriver.EncoderDirection.FORWARD,
    GoBildaPinpointDriver.EncoderDirection.REVERSED
);
```

These values depend on pod mounting orientation and must be verified on each robot.

At initial heading 0°, this controller's intended field-coordinate behavior is:

| Robot motion | Expected reported change |
| --- | --- |
| Forward | X increases |
| Backward | X decreases |
| Left | Y increases |
| Right | Y decreases |
| Counterclockwise rotation | Heading increases |

The odometry encoder axes are robot-relative measurements; Pinpoint's reported X/Y pose is field-relative.

### The issue found on the first robot

The original configuration mistakenly used:

```java
odom.setOffsets(90, -182.88, DistanceUnit.MM); // Incorrect X sign
```

During an approximately stationary 180° rotation, Pinpoint reported about **14 inches** of Y translation. The follower then issued a large lateral correction, causing the robot to move diagonally and return along a parallel, offset line.

Changing **only the X offset sign** fixed the observed sideways motion:

```java
odom.setOffsets(-90, -182.88, DistanceUnit.MM); // Correct for original robot
```

**Diagnostic principle:** If the robot physically rotates in place but the pose reports substantial translation, investigate odometry configuration before changing path-following mathematics.

## Coordinate systems and alignment

The robot uses **three related coordinate frames** plus a physical field reference. They must agree on axis directions, heading signs, units, and origin before the path follower is enabled.

### 1. Robot-relative coordinates (motor commands)

Robot-relative directions rotate **with the chassis**. In this implementation:

| Command | Positive direction |
| --- | --- |
| `forward` | Toward the robot's front |
| `strafe` | Toward the robot's right |
| `turn` | Clockwise |

For example, after the robot turns 180°, a positive `forward` command still drives toward its **front**, although that is now opposite the original field direction. These signs describe the *command interface*; individual motor direction settings must be configured so the chassis actually moves this way.

### 2. Pinpoint pose coordinates (fixed/local field frame)

Pinpoint reports the tracked robot point as `(x, y, heading)` relative to its reset origin. The reported X/Y axes **do not rotate with the robot**. Our controller assumes the following when heading is reset to 0°:

- **+X:** robot's initial forward direction.
- **+Y:** robot's initial left direction.
- **+heading:** counterclockwise, measured in degrees.

Thus, at heading **0°**, moving forward increases X and moving left increases Y. At heading **180°**, moving forward decreases X and moving left decreases Y. Heading `+180°` and `-180°` describe the same orientation.

**Important:** The raw X/Y *pod encoder readings* are not the same as the fixed-frame X/Y pose returned by Pinpoint. Pinpoint uses heading and the measured pod offsets to compute the pose.

### 3. Path-planner coordinates

`PathPlanner`, `PathSequence`, and `MecanumPathFollower` must use the **same fixed-frame coordinate convention** as Pinpoint: +X initial-forward, +Y initial-left, and positive heading counterclockwise. The demonstration OpMode shifts the Pinpoint origin by `(72, 72)` inches:

```java
Pose2D plannerPose = new Pose2D(
    pose.x + 72.0,
    pose.y + 72.0,
    pose.heading
);
```

For this demonstration, a Pinpoint pose of `(0, 0, 0°)` becomes planner pose `(72, 72, 0°)`. A 24-inch initial-forward move becomes `(96, 72, 0°)`; after rotating 180° in place, the pose should remain near `(96, 72, ±180°)`.

**Apply the origin offset exactly once**, consistently to the starting pose, live pose, and waypoints. Do not add 72 inside both the OpMode and the follower. All distances in the current planner and follower are **inches**; pod mounting offsets supplied to `setOffsets()` above are **millimeters**.

### 4. Physical FTC field coordinates

The 144 × 144-inch field is a physical space; it does not automatically establish the Pinpoint origin or heading. For this test, `(72,72)` is a **software convention**, not a guarantee that the robot is physically at field center.

To use absolute field positions, choose a documented field origin and axis orientation, physically place the robot at a known start pose, and initialize/transform the Pinpoint pose to match. A simple translation `(x+72, y+72)` works **only when the Pinpoint axes are already aligned with the chosen planner axes**. If the field axes are rotated relative to the Pinpoint reset heading, an additional **rotation transform** is required; adding an offset alone is insufficient.

### 5. Field-to-robot command transformation

The follower first computes the fixed-frame position error:

```java
double errorX = targetX - robotX;
double errorY = targetY - robotY;
```

With heading `theta` measured **counterclockwise** from +X, the intended conversion for this robot's command convention is:

```java
double theta = Math.toRadians(robotHeadingDegrees);
double forwardError = errorX * Math.cos(theta)
                    + errorY * Math.sin(theta);
double strafeError  = errorX * Math.sin(theta)
                    - errorY * Math.cos(theta); // positive = robot-right
```

This sign on `strafeError` matters: **positive field Y means left at heading 0°, but positive robot strafe means right**.

| Heading | Fixed-frame target error | Expected robot-relative correction |
| --- | --- | --- |
| 0° | `(+X, 0)` | Forward (+) |
| 0° | `(0, +Y)` | Strafe left (−) |
| 180° | `(-X, 0)` | Forward (+) |
| 180° | `(0, +Y)` | Strafe right (+) |

These are direction checks, not complete motor-power calculations; the controller also applies PD gains and output limits.

### 6. Alignment and verification on a second robot

1. **Choose the tracking point:** normally the chassis center. Measure pod offsets relative to that same point; use the Pinpoint driver's sign conventions (**X pod right = negative; Y pod rear = negative**).
2. **Set encoder directions:** move the chassis by hand, at initial heading 0°, and confirm forward gives **+X** and left gives **+Y**. Confirm a counterclockwise turn increases heading.
3. **Verify rotation compensation:** mark the chassis center and manually rotate **180°** around it. X/Y should remain close to their starting values. A 360°-only test can hide an offset error.
4. **Verify robot commands:** at low power, confirm positive `forward` moves toward the front, positive `strafe` moves right, and positive `turn` rotates clockwise. Correct motor mapping/directions if not.
5. **Verify planner origin:** after reset, check that the OpMode displays near `(72,72)` when using the +72/+72 translation; moving 12 inches initially forward should display near `(84,72)`.
6. **Verify after turning:** rotate 180° in place; the displayed planner X/Y should stay nearly unchanged. Then manually move the chassis forward: fixed-frame **X should decrease**. Move toward its left: fixed-frame **Y should decrease**.
7. **Only then run path chaining:** start with reduced translation/turn power. If the reported pose is wrong, fix localization before adjusting PD gains.

**Case study:** Our first robot's X pod was mounted **right** of center, but its X offset was configured `+90 mm`. Pinpoint then reported roughly **14 inches of false Y displacement** during a 180° turn, and the follower commanded an unnecessary strafe. Changing that offset to `-90 mm` resolved the observed behavior. The offset values themselves are robot-specific.

## Odometry verification

Use an OpMode that updates Pinpoint and displays **raw X, Y, and heading**, without commanding drive motors. Mark the robot's center on the floor. Reset pose and heading before testing.

### 1. Forward/backward

Move the robot forward ~24 inches without rotating:

```text
X ≈ +24 in
Y ≈ 0 in
Heading ≈ 0°
```

Move it back. X and Y should return near zero.

### 2. Sideways

Move the robot left ~12 inches without rotating:

```text
X ≈ 0 in
Y ≈ +12 in
Heading ≈ 0°
```

Return to the starting point and confirm the pose returns near zero.

### 3. Rotate 180° — essential test

Keep the robot's center over a floor mark while manually rotating approximately 180°:

```text
X ≈ 0 in
Y ≈ 0 in
Heading ≈ ±180°
```

A large X/Y change suggests an offset, direction, wheel-contact, or actual physical-center movement problem. This test revealed the original configuration error.

### 4. Rotate 360°

Verify the robot returns close to its original position and heading. **Do not substitute this for the 180° test:** a full rotation may conceal a compensation error by bringing the reported position back toward its starting value.

### 5. Combined motion

Drive or manually move forward, rotate 90°, then move forward again. Confirm that the second translation affects the appropriate **field-relative** coordinate.

> Small errors are normal. Investigate large errors before running autonomous path following.

## Understanding PD control

**PD** stands for **Proportional–Derivative**. It uses position error and its rate of change to calculate a correction:

\[
u(t) = K_p e(t) + K_d \frac{de(t)}{dt}
\]

For a **stationary** target, the same concept can be written as:

\[
u = K_p(\text{target} - \text{position}) - K_d(\text{velocity})
\]

- **P (`Kp`)** pushes toward the target in proportion to distance from it.
- **D (`Kd`)** damps motion, helping reduce overshoot and oscillation.

**Example:** Target X = 96 in, current X = 90 in, `Kp = 0.055`:

```text
Error = 6 in
P correction = 6 × 0.055 = 0.33
```

If the robot is already moving toward the stationary target at 10 in/s and `Kd = 0.01`:

```text
D damping = 10 × 0.01 = 0.10
PD correction ≈ 0.33 - 0.10 = 0.23
```

This illustrates the control principle; the actual path follower also uses lookahead targets, coordinate transformations, power limits, and slew-rate limits.

| Setting | Too low | Too high |
| --- | --- | --- |
| `Kp` | Sluggish response, poor correction | Overshoot, oscillation, abrupt commands |
| `Kd` | Insufficient damping | Sluggish or noisy response |

The normal path-following **translation** controller uses PD. Normal **heading** correction uses proportional control. The separate station-keeping controller has translation and heading PD terms.

## Path follower parameters

Values below are from the `MecanumPathFollower` version used during our troubleshooting. They are **starting points**, not universal settings.

| Parameter | Current value | Meaning |
| --- | ---: | --- |
| `translationKp` | 0.055 | Translation proportional gain |
| `translationKd` | 0.01 | Translation derivative gain |
| `headingKp` | 0.014 | Heading proportional gain (power/degree) |
| `maxTranslationPower` | 0.55 | Translation output cap |
| `maxTurnPower` | 0.35 | Turn output cap |
| `translationAccelRate` | 1.5 | Translation command acceleration limit |
| `translationDecelRate` | 2.5 | Translation command deceleration limit |
| `turnAccelRate` | 1.0 | Turn command acceleration limit |
| `turnDecelRate` | 4.0 | Turn command deceleration limit |
| `minLookahead` | 4 in | Minimum lookahead |
| `maxLookahead` | 10 in | Maximum lookahead |
| `positionTolerance` | 1.5 in | Position completion tolerance |
| `headingTolerance` | 4° | Heading completion tolerance |
| `passThroughRadius` | 2 in | Pass-through waypoint radius |

### Tuning procedure

1. **Verify hardware and odometry first.** Confirm forward, strafe, and turn directions; pose axes; encoder directions; and rotation compensation.
2. **Reduce power for first runs.** For example, try `maxTranslationPower = 0.30` and `maxTurnPower = 0.20` in a clear test area.
3. **Tune translation P.** Temporarily set `translationKd = 0.0`. Run a short straight segment. Increase `translationKp` gradually until the robot tracks responsively without excessive oscillation.
4. **Add translation D.** Start near `translationKd = 0.01` and adjust gradually. Increase only if more damping is needed; reduce if movement becomes sluggish or noisy.
5. **Tune heading P separately.** Run a rotation-only segment. Increase `headingKp` if turning is weak; decrease if the robot overshoots or oscillates.
6. **Increase power caps gradually.** Recheck stopping and tracking after significant speed increases.
7. **Tune acceleration/deceleration limits.** Lower acceleration rates produce gentler starts; higher deceleration rates permit faster power reductions.
8. **Tune lookahead and recovery only after basic tracking is reliable.**
9. **Tune station keeping independently.** It solves a different control problem from following a moving target.

**Change one variable at a time**, and record the before/after behavior.

## Lookahead and path recovery

### Lookahead

The follower targets a point **ahead of the robot on the planned curve**, not simply the final endpoint.

```java
minLookahead = 4;
maxLookahead = 10;
```

- **Smaller lookahead:** tighter curve tracking, but potentially more steering corrections or oscillation.
- **Larger lookahead:** smoother travel, but potentially more corner-cutting.

Start with the existing range and tune only after localization and PD behavior are sound.

### Recovery

The follower can enter a special mode when it is too far from its planned path:

```java
recoveryEnterDistance = 6.0;
recoveryExitDistance = 3.0;
recoveryExitSamples = 4;
recoveryMaxPower = 0.25;
recoveryLookahead = 3.0;
```

The separate entry and exit thresholds help prevent rapid switching between modes. Recovery can help after pushes or wheel slip, **but it cannot fix incorrect localization**.

## Station keeping

After the path chain completes, the OpMode starts a separate controller to hold the final X/Y position and heading.

| Parameter | Current value |
| --- | ---: |
| `holdPositionKp` | 0.06 |
| `holdPositionKd` | 0.01 |
| `holdHeadingKp` | 0.06 |
| `holdHeadingKd` | 0.0015 |
| `holdMaxTranslationPower` | 0.20 |
| `holdMaxTurnPower` | 0.12 |
| `holdPositionTolerance` | 0.25 in |
| `holdHeadingTolerance` | 2° |
| `angularVelocityTau` | 0.06 s |

To tune: first adjust position P for return-to-target response, then position D for damping. Tune heading P/D separately. Keep power limits conservative, and remember that noisy odometry can produce unwanted holding corrections.

## Mecanum drivetrain configuration

The follower's wheel-power mixing is:

```java
frontLeft  = forward + strafe + turn;
frontRight = forward - strafe - turn;
backLeft   = forward - strafe + turn;
backRight  = forward + strafe - turn;
```

Its command convention is:

| Command | Positive means |
| --- | --- |
| `forward` | Robot forward |
| `strafe` | Robot right |
| `turn` | Clockwise |

The original hardware configuration reversed the **left-side motors**. A new robot may require different settings.

Before enabling autonomous motion, test small commands independently:

```java
// Illustrative calls: adapt to the new robot's drive method.
drive(0.2, 0.0, 0.0); // Forward
drive(0.0, 0.2, 0.0); // Right
drive(0.0, 0.0, 0.2); // Clockwise
```

If a command moves in the wrong direction, correct the motor mapping/directions before tuning gains.

## Planner and coordinate system

The original OpMode uses:

```java
private static final double ROBOT_WIDTH = 17.0;
private static final double ROBOT_LENGTH = 15.0;

planner = new PathPlanner(
    144.0,        // Field dimension, inches
    ROBOT_WIDTH,
    ROBOT_LENGTH,
    0.0,          // Extra clearance
    2.0           // Planning grid size, inches
);
```

Measure the new robot's **full footprint**, including mechanisms that protrude. The original `0.0` clearance adds no safety margin. Consider additional clearance for obstacle avoidance.

Keep the coordinate conventions consistent:

- Pinpoint pose is field-relative.
- Forward robot movement at initial heading 0° increases field X.
- Left robot movement at initial heading 0° increases field Y.
- Robot-centric `strafe` commands are positive **right**.
- Headings must be normalized across ±180° to avoid interpreting a 2° error as 358°.

## Loop timing and rotation segments

`PathChainingTeleOP` updates Pinpoint **once per loop**, takes a pose snapshot, and passes that snapshot to the follower:

```java
robot.odom.update();
Pose2D pose = getPose();

Pose2D plannerPose = new Pose2D(
    pose.x + 72.0,
    pose.y + 72.0,
    pose.heading
);

if (follower.isFollowing()) {
    follower.update(plannerPose);
}
```

`MecanumPathFollower` does **not** call `odom.update()` internally. Keep loop timing reasonably consistent; long blocking delays and competing odometry updates can degrade velocity estimation.

A rotation-only segment changes heading while maintaining its X/Y target. The implementation includes an approximately **250 ms settling interval** after rotation before the next translation segment. This does not replace correct odometry compensation.

## Troubleshooting

| Symptom | Check first |
| --- | --- |
| Moves in wrong direction | Motor mapping, motor directions, coordinate signs |
| Moves sideways after turning | Pinpoint offset signs, measurements, encoder directions |
| Pose changes during in-place rotation | Rotation compensation, pod contact, physical center motion |
| Overshoots a destination | Translation Kp/Kd, power caps, deceleration |
| Oscillates near target | Kp too high, insufficient damping, velocity noise |
| Turns too slowly | Heading Kp and turn power cap |
| Overshoots heading | Heading Kp and turn deceleration |
| Cuts corners | Lookahead and path geometry |
| Repeated recovery mode | Odometry, path error, recovery thresholds |
| Stops short | Friction, power limits, gain, completion tolerance |
| Unexpected movement while holding | Localization drift, hold gains, velocity noise |

**Debug in this order:** physical motion → Pinpoint pose → rotation compensation → planned route → target vs. actual pose → commanded forward/strafe/turn → gains.

## New robot checklist

### A. Hardware

- [ ] Map front-left, front-right, back-left, and back-right motors.
- [ ] Verify wheel orientation and motor directions.
- [ ] Mount Pinpoint pods securely with reliable floor contact.
- [ ] Measure X and Y pod offsets from the chosen tracking point.
- [ ] Apply correct **signs** to both offsets.
- [ ] Verify encoder directions and heading sign.
- [ ] Update robot width and length in the planner.

### B. Odometry

- [ ] Reset pose and heading.
- [ ] Manually move forward/backward and verify X.
- [ ] Manually move left/right and verify Y.
- [ ] Rotate **180°** about robot center; X/Y should stay nearly constant.
- [ ] Rotate 360° and verify return near start.
- [ ] Move after a 90° rotation; verify field-coordinate mapping.

### C. Basic drivetrain

- [ ] Test low-power forward command.
- [ ] Test low-power right-strafe command.
- [ ] Test low-power clockwise-turn command.
- [ ] Confirm field-to-robot command transformation.

### D. Controller tuning

- [ ] Lower initial translation and turn power caps.
- [ ] Tune translation Kp with Kd temporarily zero.
- [ ] Add translation Kd for damping.
- [ ] Tune heading Kp on rotation-only moves.
- [ ] Increase speed caps gradually.
- [ ] Tune acceleration and deceleration.
- [ ] Tune lookahead and recovery if needed.
- [ ] Tune station keeping separately.

### E. Route verification

- [ ] Run short straight segment.
- [ ] Run rotation-only segment.
- [ ] Run forward → rotate → return chain.
- [ ] Verify final position correction.
- [ ] Test controlled displacement and recovery in a clear area.
- [ ] Save the new robot's known-good settings.

## Lessons learned

1. **Localization first.** A path follower cannot correct for false position readings.
2. **Offset signs matter.** On our first robot, a right-mounted X pod needed `-90`, not `+90` mm.
3. **A 180° rotation test is especially revealing.** Our initial 360° test concealed the issue.
4. **Do not assume sideways movement means incorrect strafe math.** The controller may be reacting correctly to an incorrect pose.
5. **Tune P before D.** Add damping after establishing stable proportional response.
6. **Keep path-following and station-keeping gains separate.**
7. **Start slowly and change one variable at a time.**
8. **Log pose, targets, and commands.** They help distinguish perception errors from control errors.
9. **Keep a robot-specific configuration record.** Offsets, encoder directions, motor directions, dimensions, and gains may all differ.

### Final principle

**Accurate localization + correct drivetrain configuration + properly tuned control parameters = reliable path chaining.**

The original sideways-motion issue was resolved by correcting the **X pod offset sign**, without changing `MecanumPathFollower`.
