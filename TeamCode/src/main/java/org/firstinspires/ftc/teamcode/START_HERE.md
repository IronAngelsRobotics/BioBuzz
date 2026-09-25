# Start Here

Welcome to the team's code. There are only five files, and you only ever edit one of
them to fix the robot.

## The files

| File | What it is |
|------|-----------|
| `Robot.java` | **The one you edit.** Motor names, motor directions, wheel measurements, and all the driving math. |
| `TestMotorDirections.java` | Day-one setup check. Run this before anything else. |
| `TeleOpDrive.java` | Driver-controlled driving for a match. |
| `AutoDriveByTime.java` | The easy autonomous. Start here. |
| `AutoDriveByEncoder.java` | The accurate autonomous. Move up to this. |

## Day one, in order

1. **Build the robot configuration** on the Driver Station with these four motor names,
   spelled exactly like this:
   `front_left_drive`, `front_right_drive`, `back_left_drive`, `back_right_drive`
2. **Run `Test: Motor Directions`.** Lift the wheels off the ground first. Confirm each
   button spins the wheel you expect, in the direction that would push the robot forward.
   Fix wrong *ports* in the robot configuration; fix wrong *directions* in `Robot.java`.
3. **Run `Drive (TeleOp)`.** Drive it around. Do not move on until this feels right.
4. **Run `Auto: Drive By Time`.** It will work immediately.
5. **Measure the wheels**, update `WHEEL_DIAMETER_INCHES` in `Robot.java`, then run
   `Auto: Drive By Encoder` and tune the turn.

## When something breaks

**Robot crashes the moment you press INIT.**
A motor name in `Robot.java` does not match the robot configuration. Check spelling,
underscores, and capitals. This is the single most common problem, all season, forever.

**Robot drives backward when you push the stick forward.**
A motor direction is wrong. Run `Test: Motor Directions`, then flip the bad motor's
`setDirection` line in `Robot.java`. You only fix it in that one place, and both TeleOp
and autonomous are fixed at the same time.

**Robot spins when you want it to go straight, or crabs sideways.**
One motor's direction is flipped relative to the others. Same fix as above.

**Autonomous goes the wrong distance.**
Check `WHEEL_DIAMETER_INCHES` in `Robot.java` against the real wheels.

**Nothing happens at all.**
Check the battery, then check that the OpMode you picked on the Driver Station is the
one you think it is.

## Why `Robot.java` exists

Look at the sample OpModes in the `FtcRobotController` module: each one sets up its own
motors and its own motor directions. That works fine for a single demo, but once you
have a TeleOp and two autonomous programs, the same wiring facts are written down four
times. Then you flip a motor direction, forget one file, and your autonomous quietly
disagrees with your TeleOp about which way is forward. Teams lose matches to this.

Everything about the physical robot lives in `Robot.java`. The OpModes only describe
what you want the robot to *do*.

## About the samples folder

The `FtcRobotController` module has about 70 sample OpModes. Almost all of them are for
sensors and features we do not have yet. **Do not edit anything in that folder** — it
gets overwritten every time we update the FTC SDK.

The four worth reading when you are curious:

- `BasicOmniOpMode_Linear` — the mecanum math that `Robot.drive()` wraps up
- `RobotAutoDriveByTime_Linear` — the same idea as our `AutoDriveByTime`, written the long way
- `RobotAutoDriveByEncoder_Linear` — same, for encoders (note: its motor numbers are for an
  old TETRIX motor, not our goBILDA ones — ours are already correct in `Robot.java`)
- `ConceptTelemetry` — how to print things to the Driver Station, which is how you debug
  everything else

## Adding an arm, claw, or intake

Add it to `Robot.java`, not to an OpMode:

1. Add a name constant next to the four motor names.
2. Add the field and look it up in the constructor.
3. Add a method that says what it does in plain language — `openClaw()`, `liftArm()`.

Then every OpMode can use it, and there is still only one file to fix when the hardware
changes.
