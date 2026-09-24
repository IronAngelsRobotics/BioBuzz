package org.firstinspires.ftc.teamcode36807.OpModes.Practice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;


/**
 * Demo for REV digital LED
 */

//@Autonomous(name="MyAutonomous")
@TeleOp(name="LED")
public class LedDemo extends LinearOpMode {
    // Declare variables
    DigitalChannel greenLED;
    DigitalChannel redLED;

    @Override
    public void runOpMode() {
        // Init hardware
        greenLED = hardwareMap.get(DigitalChannel.class, "green");
        redLED = hardwareMap.get(DigitalChannel.class, "red");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // change LED mode from input to output
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setState(true);
        redLED.setState(true);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.y){
                redLED.setState(false);
                greenLED.setState(true);
            }
            if(gamepad1.x){
                greenLED.setState(false);
                redLED.setState(true);
            }
            if(gamepad1.a){
                for(int x=0; x<5; x++) {
                    greenLED.setState(true);
                    redLED.setState(true);
                    sleep(1000);
                    redLED.setState(false);
                    greenLED.setState(true);
                }
            }
        }
    }
}
