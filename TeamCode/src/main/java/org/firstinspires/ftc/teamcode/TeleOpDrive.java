package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Driver-controlled driving. This is the OpMode you run during the teleop part of a match.
 *
 * CONTROLS (gamepad 1)
 *   Left stick            drive forward/backward and slide left/right
 *   Right stick left/right  spin left/right
 *   Right trigger         hold for precision mode (half speed)
 *
 * Notice how short this is. All the motor setup and all the mecanum math live in
 * Robot.java, so this file only has to answer one question: what do the sticks mean?
 */
@TeleOp(name = "Drive (TeleOp)", group = "Competition")
public class TeleOpDrive extends LinearOpMode {

    // Holding the right trigger scales everything down to this fraction of full speed.
    // Great for lining up carefully. Raise or lower it to taste.
    private static final double PRECISION_SPEED = 0.4;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        telemetry.addData("Status", "Ready to drive");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Read the joysticks.
            //
            // The minus sign on left_stick_y is not a typo and it is not optional.
            // The gamepad reports the stick as NEGATIVE when you push it away from you,
            // which is backwards from how people think about "forward." Flipping it here
            // means the rest of our code gets to use the sensible version: positive is
            // forward. Take the minus sign out and the robot will drive away from the
            // stick instead of with it.
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double turn    =  gamepad1.right_stick_x;

            // Precision mode: the harder you squeeze, the slower the robot goes.
            if (gamepad1.right_trigger > 0.1) {
                forward *= PRECISION_SPEED;
                strafe  *= PRECISION_SPEED;
                turn    *= PRECISION_SPEED;
            }

            robot.drive(forward, strafe, turn);

            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("Precision mode", gamepad1.right_trigger > 0.1 ? "ON" : "off");
            telemetry.update();
        }
    }
}
