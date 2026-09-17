package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * The simplest possible autonomous: turn the motors on, wait, turn them off.
 *
 * WRITE THIS ONE FIRST. There is nothing to calibrate and nothing to measure, so it
 * will work the very first time you run it. That makes it a great way to prove the
 * robot, the battery, and the Driver Station are all healthy before you start
 * debugging anything harder.
 *
 * HOW IT WORKS: sleep(2000) means "wait 2000 milliseconds" -- 2 seconds. The motors
 * keep doing whatever you last told them to do the whole time we are waiting.
 *
 * THE CATCH -- and this matters at competition:
 * Driving by time is not repeatable. A fully charged battery drives the robot noticeably
 * farther in 2 seconds than a battery at the end of a long practice. Carpet is slower
 * than tile. So you will tune this perfectly at home, and then it will overshoot or
 * come up short at the meet.
 *
 * Once this works, move up to AutoDriveByEncoder, which measures actual distance and
 * does not care about the battery.
 */
@Autonomous(name = "Auto: Drive By Time", group = "Competition")
public class AutoDriveByTime extends LinearOpMode {

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED  = 0.4;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        telemetry.addData("Status", "Ready. Autonomous by time.");
        telemetry.update();

        waitForStart();

        // Everything below happens in order, top to bottom.
        // Change the numbers to change the path.

        // Drive forward for 2 seconds.
        telemetry.addData("Step", "1 of 3 - forward");
        telemetry.update();
        robot.drive(DRIVE_SPEED, 0, 0);
        sleep(2000);

        // Slide right for 1 second. (This is the mecanum superpower -- no turning needed.)
        telemetry.addData("Step", "2 of 3 - strafe right");
        telemetry.update();
        robot.drive(0, DRIVE_SPEED, 0);
        sleep(1000);

        // Spin right for half a second.
        telemetry.addData("Step", "3 of 3 - turn right");
        telemetry.update();
        robot.drive(0, 0, TURN_SPEED);
        sleep(500);

        // ALWAYS stop at the end. If you forget this line, the robot keeps driving
        // until the OpMode is killed -- usually straight into a wall.
        robot.stop();

        telemetry.addData("Status", "Autonomous complete");
        telemetry.update();
    }
}
