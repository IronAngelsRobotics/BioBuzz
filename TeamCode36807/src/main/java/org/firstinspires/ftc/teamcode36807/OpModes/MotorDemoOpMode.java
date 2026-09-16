package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


//@Autonomous(name="MyAutonomous")
@TeleOp(name="MotorDemo")
//@Disabled   // Delete this line
public class MotorDemoOpMode extends LinearOpMode {
    // Declare variable
    DcMotor motor;
    double maxPower = .125;

    @Override
    public void runOpMode() {
        // Init hardware
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.x){
                motor.setPower(0);
            }
            else if (gamepad1.y) {
                motor.setPower(maxPower);
            }
        }
    }

    public double scale(double power){
        return (power > 1) ? maxPower : (power < -1)? -maxPower : power * maxPower;
    }
}
