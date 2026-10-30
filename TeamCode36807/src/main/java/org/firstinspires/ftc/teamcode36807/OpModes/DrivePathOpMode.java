package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode36807.Libs.MecanumDrive;
import org.firstinspires.ftc.teamcode36807.Libs.MecanumPathFollower;
import org.firstinspires.ftc.teamcode36807.Robots.BeezyBot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode36807.Libs.Pose2D;

/**
 * Use this class as a template for creating OpModes.
 */

@TeleOp(name="Pathing")
@Disabled   // Delete this line
public class DrivePathOpMode extends OpMode {

    // Movement sequence states.
    private enum DriveState {
        IDLE,
        FORWARD_24,
        ROTATE_180,
        RETURN_24,
        COMPLETE
    }

    // Current movement state.
    private DriveState driveState = DriveState.IDLE;

    // Detect a new Y-button press.
    private boolean previousY = false;

    // Store the starting position and heading.
    private Pose2D startPose;

    // Target position and heading for the current movement.
    private Pose2D targetPose;

    private static final double DISTANCE = 24.0;

    private BeezyBot robot;
    private MecanumPathFollower follower;

    @Override
    public void init() {

        // Initialize robot hardware.
        robot = new BeezyBot(hardwareMap);

        // Initialize the existing PD controller.
        follower = new MecanumPathFollower(
                robot.drive.getWheel(MecanumDrive.Wheel.FRONT_LEFT),
                robot.drive.getWheel(MecanumDrive.Wheel.FRONT_RIGHT),
                robot.drive.getWheel(MecanumDrive.Wheel.BACK_LEFT),
                robot.drive.getWheel(MecanumDrive.Wheel.BACK_RIGHT)
        );

        // Reset movement state.
        driveState = DriveState.IDLE;
        previousY = false;

        telemetry.addLine("Ready - Press Y to start");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Update Pinpoint odometry.
        robot.odom.update();

        // Read the current robot position.
        Pose2D pose = getPose();

        // Detect a new Y-button press.
        boolean yPressed = gamepad1.y && !previousY;
        previousY = gamepad1.y;

        // Start the sequence only when the robot is idle or finished.
        if (yPressed &&
                (driveState == DriveState.IDLE ||
                        driveState == DriveState.COMPLETE)) {

            // Save the starting position and heading.
            startPose = pose;

            // Calculate a target 24 inches forward.
            double headingRad = Math.toRadians(startPose.heading);

            double targetX = startPose.x
                    + DISTANCE * Math.cos(headingRad);

            double targetY = startPose.y
                    + DISTANCE * Math.sin(headingRad);

            // Set the first target.
            targetPose = new Pose2D(
                    targetX,
                    targetY,
                    startPose.heading
            );

            follower.startHolding(targetPose);

            driveState = DriveState.FORWARD_24;
        }

        // Execute the current movement using the existing PD controller.
        if (driveState == DriveState.FORWARD_24) {

            // Drive toward the target position and heading.
            follower.holdPosition(pose);

            // Check whether the robot has reached the target.
            if (follower.isAtTarget(pose)) {

                // Stop the motors before changing targets.
                follower.stop();

                // Set a new target at the same position,
                // but facing 180 degrees from the starting heading.
                targetPose = new Pose2D(
                        pose.x,
                        pose.y,
                        normalizeHeading(startPose.heading + 180.0)
                );

                // Start the heading correction.
                follower.startHolding(targetPose);

                // Advance to the rotation state.
                driveState = DriveState.ROTATE_180;
            }
        }

        // STEP 2: Rotate 180 degrees.
        else if (driveState == DriveState.ROTATE_180)  {

            // Use the existing PD controller to rotate toward the target heading.
            follower.holdPosition(pose);

            // Check whether the rotation is complete.
            if (follower.isAtTarget(pose)) {

                // Stop the motors after completing the 180° rotation.
                follower.stop();

                // Return to the original starting position,
                // while maintaining the opposite heading.
                targetPose = new Pose2D(
                        startPose.x,
                        startPose.y,
                        normalizeHeading(startPose.heading + 180.0)
                );

                // Set the return destination.
                follower.startHolding(targetPose);

                // Advance to the return movement.
                driveState = DriveState.RETURN_24;
            }
        }
        // STEP 3: Drive back to the starting position.
        else if (driveState == DriveState.RETURN_24) {

            // Drive toward the original position
            // while maintaining the opposite heading.
            follower.holdPosition(pose);

            // Check whether the robot has returned.
            if (follower.isAtTarget(pose)) {

                // Stop all drive motors.
                follower.stop();

                // Mark the sequence complete.
                driveState = DriveState.COMPLETE;
            }
        }

        telemetry.addData("State", driveState);
        telemetry.addData("X", "%.2f", pose.x);
        telemetry.addData("Y", "%.2f", pose.y);
        telemetry.addData("Heading", "%.1f", pose.heading);
        telemetry.update();
    }

    @Override
    public void stop() {

        // Stop all drive motors when the OpMode ends.
        if (follower != null) {
            follower.stop();
        }
    }



    /**
     * Read the current position from goBILDA Pinpoint.
     */
    private Pose2D getPose() {

        return new Pose2D(
                robot.odom.getPosX(DistanceUnit.INCH),
                robot.odom.getPosY(DistanceUnit.INCH),
                robot.odom.getHeading(AngleUnit.DEGREES)
        );
    }

    /**
     * Normalize heading to the range [-180, 180).
     */
    private double normalizeHeading(double heading) {

        while (heading >= 180) heading -= 360;
        while (heading < -180) heading += 360;

        return heading;
    }
}
