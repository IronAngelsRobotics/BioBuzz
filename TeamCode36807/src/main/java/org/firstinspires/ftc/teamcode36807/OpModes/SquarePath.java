package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode36807.Libs.*;
import org.firstinspires.ftc.teamcode36807.Robots.BeezyBot;

/**
 * Drives a 24-inch square without rotating the robot:
 * forward, strafe right, backward, strafe left.
 *
 * Y starts the square. The robot holds its initial pose on completion.
 * Uses the same planner coordinates and follower as PathChainingTeleOP.
 */
@TeleOp(name = "SquarePath")
public class SquarePath extends OpMode {

    private static final double SIDE_INCHES = 24.0;
    private static final double FIELD_SIZE = 144.0;
    private static final double FIELD_ORIGIN = 72.0;

    private BeezyBot robot;
    private MecanumPathFollower follower;
    private PathPlanner planner;
    private PlannedRoute currentRoute;
    private Pose2D startingPose;

    private boolean previousY;
    private boolean holding;

    @Override
    public void init() {
        robot = new BeezyBot(hardwareMap);
        robot.odom.resetPosAndIMU();

        follower = new MecanumPathFollower(
                robot.drive.getWheel(MecanumDrive.Wheel.FRONT_LEFT),
                robot.drive.getWheel(MecanumDrive.Wheel.FRONT_RIGHT),
                robot.drive.getWheel(MecanumDrive.Wheel.BACK_LEFT),
                robot.drive.getWheel(MecanumDrive.Wheel.BACK_RIGHT)
        );

        // Keep the known-good follower gains and tolerances unchanged.
        planner = new PathPlanner(FIELD_SIZE, robot.size.width, robot.size.length, 0.0, 2.0);

        telemetry.addLine("SquarePath ready - press Y to start");
        telemetry.addLine("Forward > Right > Backward > Left (24 in each)");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.odom.update();

        // Pinpoint (odometry) origin is (0,0); planner origin is (72,72).
        Pose2D pose = new Pose2D(
                robot.odom.getPosX(DistanceUnit.INCH) + FIELD_ORIGIN,
                robot.odom.getPosY(DistanceUnit.INCH) + FIELD_ORIGIN,
                robot.odom.getHeading(AngleUnit.DEGREES)
        );

        boolean yPressed = gamepad1.y && !previousY;
        previousY = gamepad1.y;

        // Only allow one run per OpMode initialization. This avoids
        // silently redefining the original starting point after holding.
        if (yPressed && startingPose == null && !follower.isFollowing()) {
            startSquare(pose);
        }

        if (follower.isFollowing()) {
            follower.update(pose);
            if (!follower.isFollowing() && startingPose != null) {
                follower.startHolding(startingPose);
                holding = true;
            }
        }

        if (holding) {
            follower.holdPosition(pose);
        }

        telemetry.addData("State", follower.isFollowing() ? "Following"
                : holding ? "Holding" : "Ready");
        telemetry.addData("Segment", follower.segmentIndex());
        telemetry.addData("Position (in)", "X %.2f  Y %.2f", pose.x, pose.y);
        telemetry.addData("Heading (deg)", "%.1f", pose.heading);
        if (startingPose != null) {
            double errorX = startingPose.x - pose.x;
            double errorY = startingPose.y - pose.y;
            telemetry.addData("Start error X (in)", "%.3f", errorX);
            telemetry.addData("Start error Y (in)", "%.3f", errorY);
            telemetry.addData("Start error total (in)", "%.3f",
                    Math.hypot(errorX, errorY));
        }
        if (currentRoute != null) {
            telemetry.addData("Route segments", currentRoute.segments().size());
            telemetry.addData("Target", "X %.2f  Y %.2f",
                    follower.getTargetX(), follower.getTargetY());
        }
        telemetry.update();
    }

    private void startSquare(Pose2D start) {
        // The square is relative to the robot's heading at the start.
        // In the existing coordinate system, forward at 0 deg is +X,
        // and right at 0 deg is -Y.
        double radians = Math.toRadians(start.heading);
        double forwardX = Math.cos(radians) * SIDE_INCHES;
        double forwardY = Math.sin(radians) * SIDE_INCHES;
        double rightX = Math.sin(radians) * SIDE_INCHES;
        double rightY = -Math.cos(radians) * SIDE_INCHES;

        double x1 = start.x + forwardX;
        double y1 = start.y + forwardY;
        double x2 = x1 + rightX;
        double y2 = y1 + rightY;
        double x3 = start.x + rightX;
        double y3 = start.y + rightY;

        PathSequence sequence = new PathSequence()
                .add(x1, y1, start.heading, start.heading, WaypointMode.STOP)
                .add(x2, y2, start.heading, start.heading, WaypointMode.STOP)
                .add(x3, y3, start.heading, start.heading, WaypointMode.STOP)
                .add(start.x, start.y, start.heading, start.heading, WaypointMode.STOP);

        PlannedRoute route = planner.plan(start, sequence, start.heading);
        if (route == null || route.isEmpty()) {
            telemetry.addLine("Square path planning failed; check field clearance.");
            return;
        }

        startingPose = new Pose2D(start.x, start.y, start.heading);
        currentRoute = route;
        holding = false;
        follower.start(route);
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
        }
    }
}
