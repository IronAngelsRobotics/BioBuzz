package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode36807.Libs.MecanumDrive;
import org.firstinspires.ftc.teamcode36807.Libs.MecanumPathFollower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode36807.Libs.Pose2D;
import org.firstinspires.ftc.teamcode36807.Robots.BeezyBot;

/**
 * Demonstrate that the robot hold its position:
 * Stay at position.
 * Detect when pushes away.
 * Automatically drive back to its saved position.
 * Correct its heading if it gets rotated.
 */

@TeleOp(name="Stay")
//@Disabled   // Delete this line
public class HoldPositionOpMode extends LinearOpMode {
    BeezyBot robot;
    private MecanumPathFollower follower;


    @Override
    public void runOpMode() {
        robot = new BeezyBot(hardwareMap);
        robot.odom.resetPosAndIMU(); // reset odometry position and heading


        // -----------------------------------------------------
        // Initialize follower
        // -----------------------------------------------------

        follower = new MecanumPathFollower(
                        robot.drive.getWheel(MecanumDrive.Wheel.FRONT_LEFT),
                        robot.drive.getWheel(MecanumDrive.Wheel.FRONT_RIGHT),
                        robot.drive.getWheel(MecanumDrive.Wheel.BACK_LEFT),
                        robot.drive.getWheel(MecanumDrive.Wheel.BACK_RIGHT));


        // -----------------------------------------------------
        // Instructions
        // -----------------------------------------------------

        telemetry.addLine("A = Hold current position");
        telemetry.addLine("B = Disable station keeping");
        telemetry.addLine("Left stick = Manual driving");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        boolean previousA = false;
        boolean previousB = false;

        try {
            while (opModeIsActive()) {

                // ---------------------------------------------
                // Update Pinpoint once
                // ---------------------------------------------

                robot.odom.update();

                Pose2D pose =
                        new Pose2D(

                                robot.odom.getPosX(
                                        DistanceUnit.INCH),

                                robot.odom.getPosY(
                                        DistanceUnit.INCH),

                                robot.odom.getHeading(
                                        AngleUnit.DEGREES));


                // ---------------------------------------------
                // Button edge detection
                // ---------------------------------------------

                boolean aPressed =
                        gamepad1.a && !previousA;

                boolean bPressed =
                        gamepad1.b && !previousB;

                previousA = gamepad1.a;
                previousB = gamepad1.b;


                // ---------------------------------------------
                // A: Capture and hold position
                // ---------------------------------------------

                if (aPressed) {
                    follower.startHolding(pose);
                }

                // ---------------------------------------------
                // B: Disable holding
                // ---------------------------------------------
                if (bPressed) {

                    follower.stopHolding();
                }

                // ---------------------------------------------
                // Manual driving
                // ---------------------------------------------

                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;


                boolean manualInput =
                        Math.abs(forward) > 0.15
                                || Math.abs(strafe) > 0.15
                                || Math.abs(turn) > 0.15;


                if (manualInput) {
                    // Driver takes control
                    if (follower.isHoldingPosition()) {
                        follower.stopHolding();
                    }

                    follower.driveRobotCentric(forward, strafe, turn);
                } else if (follower.isHoldingPosition())
                {

                    // Automatically maintain position
                    follower.holdPosition(pose);
                    telemetry.addData("X", "%.2f", Math.abs(pose.x) < 0.005 ? 0.0 : pose.x);
                    telemetry.addData("Y", "%.2f", pose.y);

                    // Disable movement during diagnostic test
                    //follower.stop();

                    telemetry.addData("Error X", "%.3f", follower.getHoldErrorX(pose));
                    telemetry.addData("Error Y", "%.3f", follower.getHoldErrorY(pose));
                    telemetry.addData("Heading Error", "%.3f", follower.getHoldHeadingError(pose));

                } else {
                    follower.stop();
                }

                // ---------------------------------------------
                // Telemetry
                // ---------------------------------------------
                telemetry.addData("Holding", follower.isHoldingPosition());
                telemetry.addData("X", "%.2f", pose.x);
                telemetry.addData("Y", "%.2f", pose.y);
                telemetry.addData("Heading", "%.2f", pose.heading);
                telemetry.update();

                idle();
            }

        } finally {
            follower.stopHolding();
        }
    }
}
