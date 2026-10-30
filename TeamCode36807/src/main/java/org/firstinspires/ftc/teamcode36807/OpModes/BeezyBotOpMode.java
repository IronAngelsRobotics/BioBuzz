package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode36807.Robots.BeezyBot;


/**
 * This opmode tests that the GoBilda starter bot is functioning correctly.
 */

//@Autonomous(name="MyAutonomous")
@TeleOp(name="Beezy")
//@Disabled
public class BeezyBotOpMode extends LinearOpMode {
    // Declare variables
    BeezyBot robot;
    boolean previousLeftTrigger = false; // toggle flywheel & middle servo
    boolean previousRightTrigger = false; // toggle intake
    boolean previousX = false;
    boolean previousA = false;
    boolean previousY = false;
    double currentFlywheelPower = 0.5;
    double intakeMaxPower = .45;
    private static final double funnelUpPosition = 0.45;
    private static final double funnerlDownPostion = 0.565;

    @Override
    public void runOpMode() {
        robot = new BeezyBot(hardwareMap);
        robot.servoLift.setPosition(funnerlDownPostion); // lower funnel

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.rgb.green();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double turn    = gamepad1.right_stick_x;
            boolean currentLeftTrigger = gamepad1.left_trigger_pressed;
            boolean currentRightTrigger = gamepad1.right_trigger_pressed;
            boolean currentX = gamepad1.x;
            boolean currentA = gamepad1.a;
            boolean currentY= gamepad1.y;


            // Set power to wheels
            robot.drive.setPower(forward, strafe, turn);

            // Press left trigger to start flywheel, stop intake, and raise funnel.
            // Or, reset flywheel, funnel and leave funnel down.
            if (currentLeftTrigger && !previousLeftTrigger)
            {
                double power = robot.flywheel.getPower();
                if(power == 0){
                    robot.flywheel.setPower(currentFlywheelPower);
                    robot.intake.setPower(0);
                    sleep(500);
                    robot.servoLift.setPosition(funnelUpPosition);
                }
                else{
                    robot.flywheel.setPower(0);
                    robot.servoLift.setPosition(funnerlDownPostion);
                    robot.intake.setPower(intakeMaxPower);
                }
            }
            // Press Y button to spin intake in the reverse direction.
            else if(currentY && !previousY){
                // reverse spinning direction of intake.
                robot.intake.setPower(0);
                robot.intake.setPower(-intakeMaxPower);
            }
            // Toggle intake on/off
            else if(currentRightTrigger && !previousRightTrigger){
                double power = robot.intake.getPower() != 0 ? 0 : intakeMaxPower;
                robot.intake.setPower(power);
            }

            // increase speed of flywheel
            else if(currentX && !previousX){
                double new_power = robot.flywheel.getPower() + .1;
                double power = (new_power > 1) ? 1 : new_power;
                robot.flywheel.setPower(power);
                currentFlywheelPower = power;
            }
            // decrease speed of flywheel
            else if(currentA && !previousA){
                double new_power = robot.flywheel.getPower() - .1;
                double power = (new_power < 0) ? 0 : new_power;
                robot.flywheel.setPower(power);
                currentFlywheelPower = power;
            }

            previousLeftTrigger = currentLeftTrigger;
            previousRightTrigger = currentRightTrigger;
            previousA = currentA;
            previousX = currentX;
            previousY = currentY;

            telemetry.addData("flywheel power: ", robot.flywheel.getPower());
            telemetry.update();
        }

        robot.drive.stop();
    }
}
