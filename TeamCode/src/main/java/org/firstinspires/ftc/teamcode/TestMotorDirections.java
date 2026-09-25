package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * RUN THIS FIRST, on day one, before TeleOp and before autonomous.
 *
 * It spins one wheel at a time so you can confirm two separate things:
 *
 *   1. IS THE RIGHT MOTOR PLUGGED INTO THE RIGHT PORT?
 *      Press X. If the wheel that spins is not the front-left wheel, then the robot
 *      configuration on the Driver Station has the ports mixed up. Fix it there.
 *
 *   2. IS THE MOTOR SPINNING THE RIGHT WAY?
 *      Each wheel should spin the direction that would push the robot FORWARD.
 *      If a wheel spins backward, open Robot.java and flip that motor's setDirection
 *      line between REVERSE and FORWARD.
 *
 * Do these in order -- ports first, then directions. Fixing directions while the ports
 * are still swapped will just chase you in circles.
 *
 * TIP: put the robot up on a block so the wheels can spin freely in the air.
 *
 * CONTROLS
 *   X  front-left wheel
 *   Y  front-right wheel
 *   A  back-left wheel
 *   B  back-right wheel
 */
@TeleOp(name = "Test: Motor Directions", group = "Setup")
public class TestMotorDirections extends LinearOpMode {

    // Slow, so it is easy to see which way the wheel is going and nothing runs away.
    private static final double TEST_SPEED = 0.3;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        telemetry.addData("Status", "Lift the wheels off the ground, then press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double frontLeft  = gamepad1.x ? TEST_SPEED : 0;
            double frontRight = gamepad1.y ? TEST_SPEED : 0;
            double backLeft   = gamepad1.a ? TEST_SPEED : 0;
            double backRight  = gamepad1.b ? TEST_SPEED : 0;

            robot.setIndividualPowers(frontLeft, frontRight, backLeft, backRight);

            telemetry.addLine("Each wheel should spin the way that drives the robot FORWARD.");
            telemetry.addLine();
            telemetry.addData("X - front left", gamepad1.x ? "SPINNING" : "-");
            telemetry.addData("Y - front right", gamepad1.y ? "SPINNING" : "-");
            telemetry.addData("A - back left", gamepad1.a ? "SPINNING" : "-");
            telemetry.addData("B - back right", gamepad1.b ? "SPINNING" : "-");
            telemetry.addLine();
            telemetry.addLine("Wrong wheel moved?  -> fix the robot configuration");
            telemetry.addLine("Right wheel, wrong way? -> flip setDirection in Robot.java");
            telemetry.update();
        }
    }
}
