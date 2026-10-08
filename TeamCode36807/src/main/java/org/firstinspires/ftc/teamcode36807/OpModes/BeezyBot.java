package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode36807.Libs.MecanumDrive;
import org.firstinspires.ftc.teamcode36807.Libs.RGBIndicator;


/**
 * This opmode tests that the GoBilda stater bot is functioning correctly.
 */

//@Autonomous(name="MyAutonomous")
@TeleOp(name="Beezy")
//@Disabled
public class BeezyBot extends LinearOpMode {
    // Declare variables
    DcMotor flywheel= null;
    DcMotor intake;
    CRServo servoMiddle;
    CRServo servoRight;
    CRServo servoLeft;
    boolean previousLeftTrigger = false; // toggle flywheel & middle servo
    boolean previousRightTrigger = false; // toggle intake
    RGBIndicator rgb;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        flywheel = hardwareMap.get(DcMotor.class, "flyWheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        servoMiddle = hardwareMap.get(CRServo.class, "servoMiddle");
        servoLeft = hardwareMap.get(CRServo.class, "servoLeft");
        servoRight = hardwareMap.get(CRServo.class, "servoRight");
        rgb = new RGBIndicator(hardwareMap, "rgb");

        // set motors off
        drive.stop();
        flywheel.setPower(0);
        intake.setPower(0);

        // configure motors
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        rgb.green();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double turn    = gamepad1.right_stick_x;
            boolean currentLeftTrigger = gamepad1.left_trigger_pressed;
            boolean currentRightTrigger = gamepad1.right_trigger_pressed;

            drive.drive(forward, strafe, turn);
            if (currentLeftTrigger && !previousLeftTrigger)
            {
                if(flywheel.getPower() != 0) {
                    flywheel.setPower(0);
                    servoMiddle.setPower(0);
                }
                else {
                    flywheel.setPower(.7);
                    sleep(1000);
                    servoMiddle.setPower(-1.0);
                }
            }
            else if(currentRightTrigger && !previousRightTrigger){

                if(intake.getPower() != 0) {
                    intake.setPower(0);
                    servoRight.setPower(0);
                    servoLeft.setPower(0);
                }
                else {
                    intake.setPower(.8);
                    servoRight.setPower(-1.0);
                    servoLeft.setPower(1.0);
                }
            }

            previousLeftTrigger = currentLeftTrigger;
            previousRightTrigger = currentRightTrigger;

            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.update();
        }
        drive.stop();
        flywheel.setPower(0);
        intake.setPower(0);
        rgb.off();
    }
}
