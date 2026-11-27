package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode36807.Libs.MecanumDrive;
import org.firstinspires.ftc.teamcode36807.Libs.RGBIndicator;
import org.firstinspires.ftc.teamcode36807.Robots.MentorBot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Opmode to drive the MentorBot..
 */

//@Autonomous(name="MyAutonomous")
@TeleOp(name="MentorBot")
@Disabled
public class MentorBotOpMode extends LinearOpMode {
    // Declare variables
    MentorBot robot;
    boolean previousRightTrigger = false; // toggle intake


    @Override
    public void runOpMode() {
        robot = new MentorBot(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double turn    = gamepad1.right_stick_x;
            boolean currentRightTrigger = gamepad1.right_trigger_pressed;

            robot.setPower(forward, strafe, turn);

            if(currentRightTrigger && !previousRightTrigger) {

                if (robot.intake.getPower() != 0)
                    robot.intake.setPower(0);
                else
                    robot.intake.setPower(1);
            }
            else if(gamepad1.y){
                robot.intake.setPower(0);
                robot.intake.setPower(-1);
            }

            previousRightTrigger = currentRightTrigger;

            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.update();
        }
        robot.stop();

    }
}
