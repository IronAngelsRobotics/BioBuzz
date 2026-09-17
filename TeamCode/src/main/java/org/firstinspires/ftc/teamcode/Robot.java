package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This is our robot.
 *
 * Every OpMode we write starts with one line:
 *
 *     Robot robot = new Robot(this);
 *
 * ...and after that, the OpMode just tells the robot what to do. It never has to
 * know which motor is plugged into which port, or which motors are mounted backward.
 * All of that lives in THIS file, in ONE place.
 *
 * WHY THIS FILE EXISTS:
 * The sample OpModes in the FtcRobotController folder each set up their own motors.
 * That means if you rewire the robot, or discover a motor is spinning backward, you
 * have to remember to fix it in every OpMode you have written. Teams lose whole
 * practices to "autonomous thinks forward is backward." Here, you fix it once.
 *
 * WHEN TO EDIT THIS FILE:
 *   - A motor spins the wrong way            -> the setDirection lines below
 *   - You renamed something in the robot
 *     configuration on the Driver Station    -> the four name strings below
 *   - You changed motors or wheels           -> the measurement numbers below
 */
public class Robot {

    // ---------------------------------------------------------------------------------
    // 1. NAMES
    //
    // These strings must match the names you typed into the robot configuration
    // on the Driver Station, EXACTLY. Same spelling, same underscores, no capitals.
    // If the robot crashes the instant you press INIT, it is almost always because
    // one of these four names does not match the configuration.
    // ---------------------------------------------------------------------------------
    private static final String FRONT_LEFT_NAME  = "front_left_drive";
    private static final String FRONT_RIGHT_NAME = "front_right_drive";
    private static final String BACK_LEFT_NAME   = "back_left_drive";
    private static final String BACK_RIGHT_NAME  = "back_right_drive";

    // ---------------------------------------------------------------------------------
    // 2. MEASUREMENTS  (these only matter for autonomous)
    //
    // COUNTS_PER_MOTOR_REV is a property of the motor. 537.7 is correct for a
    // goBILDA 5202/5203 Yellow Jacket at 312 RPM, which is what we are running.
    // If we ever switch to a different RPM Yellow Jacket, this number changes --
    // look it up on the goBILDA product page for the exact motor.
    //
    // ** WHEEL_DIAMETER_INCHES: MEASURE OUR ACTUAL WHEELS BEFORE TRUSTING AUTONOMOUS. **
    // 3.78 inches is a 96mm goBILDA mecanum wheel. If ours are a different size, every
    // autonomous distance will be wrong by the same percentage.
    // ---------------------------------------------------------------------------------
    private static final double COUNTS_PER_MOTOR_REV  = 537.7;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;
    private static final double GEAR_REDUCTION        = 1.0;   // 1.0 means the motor drives the wheel directly

    // How many encoder counts the robot travels per inch. This is just the two numbers
    // above combined -- you should not need to change this line itself.
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Mecanum wheels are less efficient sideways than forward: some of the motor's
    // effort gets lost pushing the rollers instead of the robot. So a sideways move
    // needs a bit of extra distance to actually travel the inches you asked for.
    // 1.15 is a reasonable starting guess. Tune it on the field: ask for 24 inches,
    // measure what you actually got, and adjust.
    private static final double STRAFE_CORRECTION = 1.15;

    // Turning cannot be measured in inches, so we count per degree instead.
    // There is no formula for this that is worth teaching in year one -- it depends on
    // how wide the robot is and how much the wheels scrub on the floor.
    // TUNE IT: ask for a 90 degree turn, see what you get, scale this number.
    // (Turned too far? Make it smaller. Not far enough? Make it bigger.)
    private static final double COUNTS_PER_DEGREE = 9.0;

    // If an autonomous move somehow gets stuck, give up after this long and move on
    // rather than burning the whole 30 second autonomous period in one step.
    private static final double DEFAULT_TIMEOUT_SECONDS = 4.0;

    // ---------------------------------------------------------------------------------
    // The parts
    // ---------------------------------------------------------------------------------
    private final LinearOpMode opMode;
    private final ElapsedTime timer = new ElapsedTime();

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    // The same four motors again, in a list, for when we want to do the same thing to
    // all of them ("stop everybody", "reset everybody").
    private final DcMotor[] allMotors;

    /*
     * Builds the robot and gets it ready to drive.
     * The "this" you pass in is the OpMode itself -- that is how the robot gets access
     * to the hardware list and to the STOP button.
     */
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;

        // Look up each motor by the name from the robot configuration.
        frontLeft  = opMode.hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRight = opMode.hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeft   = opMode.hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRight  = opMode.hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        allMotors = new DcMotor[] { frontLeft, frontRight, backLeft, backRight };

        // #############################################################################
        // MOTOR DIRECTIONS -- the most important four lines in this file.
        //
        // The motors on the left and right sides of the robot are mirror images of each
        // other, so "spin forward" for one side is "spin backward" for the other. We fix
        // that here, once, so that positive power ALWAYS means forward for every motor.
        //
        // HOW TO CHECK: run TestMotorDirections. Each button should spin one wheel so
        // that it would push the robot FORWARD. If a wheel spins the wrong way, flip
        // that motor's line below between REVERSE and FORWARD.
        // #############################################################################
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Stop hard instead of coasting when we cut power. Makes the robot much easier
        // to drive accurately, and makes autonomous distances more repeatable.
        for (DcMotor motor : allMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        resetEncoders();
    }

    // =================================================================================
    // DRIVING BY HAND  (this is what TeleOp uses)
    // =================================================================================

    /*
     * Drive the robot. All three at once is fine -- that is the whole point of mecanum.
     *
     *   forward  1.0 = full speed forward,   -1.0 = full speed backward
     *   strafe   1.0 = full speed right,     -1.0 = full speed left
     *   turn     1.0 = spin right (clockwise from above), -1.0 = spin left
     *
     * Mecanum wheels work because each wheel pushes the robot diagonally. Add the three
     * requests together per wheel and the diagonals cancel out into the motion you want.
     * You do not have to understand the four lines below to use this method.
     */
    public void drive(double forward, double strafe, double turn) {
        double frontLeftPower  = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower   = forward - strafe + turn;
        double backRightPower  = forward + strafe - turn;

        // Adding three numbers together can total more than 1.0, and a motor cannot go
        // faster than 1.0. If we just chopped the too-big numbers off at 1.0, the wheels
        // would be out of proportion with each other and the robot would drift off course.
        // Instead, if anything is over 1.0, shrink ALL four by the same amount. The robot
        // goes slower, but in exactly the direction the driver asked for.
        double largest = 1.0;
        largest = Math.max(largest, Math.abs(frontLeftPower));
        largest = Math.max(largest, Math.abs(frontRightPower));
        largest = Math.max(largest, Math.abs(backLeftPower));
        largest = Math.max(largest, Math.abs(backRightPower));

        frontLeft.setPower(frontLeftPower / largest);
        frontRight.setPower(frontRightPower / largest);
        backLeft.setPower(backLeftPower / largest);
        backRight.setPower(backRightPower / largest);
    }

    /* Cut power to all four wheels. */
    public void stop() {
        drive(0, 0, 0);
    }

    // =================================================================================
    // DRIVING BY ITSELF  (this is what Autonomous uses)
    //
    // Each of these three methods BLOCKS: it does not return until the robot has
    // finished the move, run out of time, or you pressed STOP. So an autonomous
    // program reads top to bottom like a list of instructions.
    // =================================================================================

    /* Drive straight. Positive inches = forward, negative = backward. */
    public void driveInches(double inches, double speed) {
        int counts = (int) (inches * COUNTS_PER_INCH);
        runToTargets(counts, counts, counts, counts, speed, DEFAULT_TIMEOUT_SECONDS);
    }

    /* Slide sideways without turning. Positive inches = right, negative = left. */
    public void strafeInches(double inches, double speed) {
        int counts = (int) (inches * COUNTS_PER_INCH * STRAFE_CORRECTION);
        // Front-left and back-right spin one way, the other two spin the other way.
        // That is what makes the diagonal pushes add up to a sideways slide.
        runToTargets(counts, -counts, -counts, counts, speed, DEFAULT_TIMEOUT_SECONDS);
    }

    /* Spin in place. Positive degrees = right / clockwise, negative = left. */
    public void turnDegrees(double degrees, double speed) {
        int counts = (int) (degrees * COUNTS_PER_DEGREE);
        // Left side forward and right side backward makes the robot spin right.
        runToTargets(counts, -counts, counts, -counts, speed, DEFAULT_TIMEOUT_SECONDS);
    }

    // =================================================================================
    // Everything below here is plumbing. You should not need to read or change it
    // in your first season -- but it is here, and it is not magic, if you get curious.
    // =================================================================================

    /*
     * Tells each motor "go exactly this many counts from where you are now," then waits.
     *
     * The motor controller does the hard part. We just hand it a target and watch.
     * Each motor has an encoder -- a sensor that counts how far the shaft has turned --
     * so the motor itself can tell when it has arrived.
     */
    private void runToTargets(int frontLeftCounts, int frontRightCounts,
                              int backLeftCounts, int backRightCounts,
                              double speed, double timeoutSeconds) {

        // If the driver already pressed STOP, do nothing at all.
        if (!opMode.opModeIsActive()) {
            return;
        }

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + frontLeftCounts);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + frontRightCounts);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + backLeftCounts);
        backRight.setTargetPosition(backRight.getCurrentPosition() + backRightCounts);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Math.abs because in RUN_TO_POSITION the target decides the direction, not the
        // sign of the power. Handing it a negative speed here would just confuse it.
        double power = Math.abs(speed);
        for (DcMotor motor : allMotors) {
            motor.setPower(power);
        }

        // Wait here until the move is done. We stop waiting if ANY of three things happen:
        //   1. all four motors report they have arrived
        //   2. we run out of time (something is stuck or blocked)
        //   3. the driver presses STOP
        timer.reset();
        while (opMode.opModeIsActive()
                && timer.seconds() < timeoutSeconds
                && anyMotorStillMoving()) {

            opMode.telemetry.addData("Moving", "%.1f of %.1f seconds",
                    timer.seconds(), timeoutSeconds);
            opMode.telemetry.addData("Wheel counts", "%d  %d  %d  %d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                    backLeft.getCurrentPosition(), backRight.getCurrentPosition());
            opMode.telemetry.update();
        }

        // Always come to a full stop before the next instruction, so each move starts
        // from a known standstill instead of coasting into the next one.
        for (DcMotor motor : allMotors) {
            motor.setPower(0);
        }
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean anyMotorStillMoving() {
        for (DcMotor motor : allMotors) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }

    private void resetEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setMode(DcMotor.RunMode mode) {
        for (DcMotor motor : allMotors) {
            motor.setMode(mode);
        }
    }

    // ---------------------------------------------------------------------------------
    // Used only by TestMotorDirections. Normal OpModes should call drive() instead.
    // ---------------------------------------------------------------------------------
    void setIndividualPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
