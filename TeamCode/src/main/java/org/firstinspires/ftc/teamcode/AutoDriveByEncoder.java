package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Autonomous that drives in real distances instead of seconds.
 *
 * Every motor has an encoder: a sensor that counts how far the shaft has actually
 * turned. That means the robot can tell how far it really went, rather than guessing
 * from how long the motors were on. A tired battery makes this autonomous SLOWER,
 * but it still ends up in the same place. That is why competition autonomous uses
 * encoders and not a stopwatch.
 *
 * BEFORE THIS WILL BE ACCURATE, do these two things once:
 *
 *   1. MEASURE THE WHEELS. Open Robot.java and check WHEEL_DIAMETER_INCHES against
 *      the wheels actually bolted to our robot. If that number is wrong, every
 *      distance below is wrong by the same percentage.
 *
 *   2. TUNE THE TURN. Run this, watch the 90 degree turn, and measure what you got.
 *      Adjust COUNTS_PER_DEGREE in Robot.java until 90 means 90. Turned too far?
 *      Make the number smaller. Not far enough? Make it bigger.
 *
 * Do #1 before #2 -- tuning turns on top of a wrong wheel size means doing it twice.
 */
@Autonomous(name = "Auto: Drive By Encoder", group = "Competition")
public class AutoDriveByEncoder extends LinearOpMode {

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED  = 0.4;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        telemetry.addData("Status", "Ready. Autonomous by encoder.");
        telemetry.update();

        waitForStart();

        // Each line below finishes completely before the next one starts, so this reads
        // like a recipe. Negative numbers mean the opposite direction:
        //   driveInches   positive = forward,   negative = backward
        //   strafeInches  positive = right,     negative = left
        //   turnDegrees   positive = clockwise, negative = counter-clockwise

        telemetry.addData("Step", "1 of 4 - forward 24 inches");
        telemetry.update();
        robot.driveInches(24, DRIVE_SPEED);

        telemetry.addData("Step", "2 of 4 - strafe right 12 inches");
        telemetry.update();
        robot.strafeInches(12, DRIVE_SPEED);

        telemetry.addData("Step", "3 of 4 - turn right 90 degrees");
        telemetry.update();
        robot.turnDegrees(90, TURN_SPEED);

        telemetry.addData("Step", "4 of 4 - back up 6 inches");
        telemetry.update();
        robot.driveInches(-6, DRIVE_SPEED);

        // Each move already stops itself, but ending with a stop is a good habit and
        // costs nothing.
        robot.stop();

        telemetry.addData("Status", "Autonomous complete");
        telemetry.update();
    }
}
