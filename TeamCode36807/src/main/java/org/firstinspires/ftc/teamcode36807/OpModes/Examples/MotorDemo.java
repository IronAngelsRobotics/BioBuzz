package org.firstinspires.ftc.teamcode36807.OpModes.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Simple OpModes to control a motor. Expect motor named 'motor'
 */


//@Autonomous(name="MyAutonomous")
@TeleOp(name="MotorDemo")
@Disabled   // comment out this line
public class MotorDemo extends LinearOpMode {
    // Declare variable
    DcMotor motor;
    double maxPower = .7;

    @Override
    public void runOpMode() {
        // Init hardware
        motor = hardwareMap.get(DcMotor.class, "frontLeft");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.x){
                // stop motor
                setPower(0);
            }
            else if (gamepad1.y) {
                // motor should rotate in the forward direction (relative to robot)
                setPower(maxPower);
            }
            else if (gamepad1.b) {
                // motor should rotate in the backward direction (relative to robot)
                setPower(-maxPower);
            }
        }
    }

    public void setPower(double power){
        double normalize_power = (power > 1) ? maxPower : (power < -1)? -maxPower : power * maxPower;
        motor.setPower(normalize_power);
    }
}
