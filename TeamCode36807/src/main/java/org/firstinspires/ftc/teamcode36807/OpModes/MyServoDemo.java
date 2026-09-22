package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Simple OpModes to control a servo.
 */


@TeleOp(name="MyServo")
public class MyServoDemo extends LinearOpMode {
    // Declare variables

    @Override
    public void runOpMode() {
        // Init hardware
        Servo servo;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Do something
            servo = hardwareMap.get(Servo.class, "servo");

            if(gamepad1.y){
                servo.setPosition(1);
            }
            if(gamepad1.x){
                // reserve servo direction
            }
        }
    }
}
