package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode36807.Libs.*;
import org.firstinspires.ftc.teamcode36807.Robots.BeezyBot;

/**
 * Drives a diamond using four diagonal movements without rotating:
 * forward-right, backward-right, backward-left, forward-left.
 * Each diagonal changes forward and strafe coordinates by 12 inches.
 *
 * Y starts the diamond. The robot holds its initial pose on completion.
 * Uses the same planner coordinates and follower as PathChainingTeleOP.
 */
@TeleOp(name = "DiagonalPath")
public class DiagonalPath extends OpMode {

    private static final double AXIS_STEP_INCHES = 12.0;
    private static final double FIELD_SIZE = 144.0;
    private static final double ROBOT_WIDTH = 17.0;
    private static final double ROBOT_LENGTH = 15.0;
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
        planner = new PathPlanner(
                FIELD_SIZE, ROBOT_WIDTH, ROBOT_LENGTH,
                0.0, 2.0
        );

        telemetry.addLine("DiagonalPath ready - press Y to start");
        telemetry.addLine("Forward-right > Back-right > Back-left > Forward-left");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.odom.update();

        // Pinpoint origin is (0,0); planner origin is (72,72).
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
            startDiamond(pose);
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

    private void startDiamond(Pose2D start) {
        // The diamond is relative to the initial robot heading.
        // At 0 degrees: forward is +X, right is -Y.
        // Each diagonal leg travels sqrt(12^2 + 12^2) = 16.97 inches.
        double radians = Math.toRadians(start.heading);
        double forwardX = Math.cos(radians) * AXIS_STEP_INCHES;
        double forwardY = Math.sin(radians) * AXIS_STEP_INCHES;
        double rightX = Math.sin(radians) * AXIS_STEP_INCHES;
        double rightY = -Math.cos(radians) * AXIS_STEP_INCHES;

        // 1: forward-right; 2: backward-right;
        // 3: backward-left; 4: forward-left (back to start).
        double x1 = start.x + forwardX + rightX;
        double y1 = start.y + forwardY + rightY;
        double x2 = start.x + 2.0 * rightX;
        double y2 = start.y + 2.0 * rightY;
        double x3 = start.x - forwardX + rightX;
        double y3 = start.y - forwardY + rightY;

        PathSequence sequence = new PathSequence()
                .add(x1, y1, start.heading, start.heading, WaypointMode.STOP)
                .add(x2, y2, start.heading, start.heading, WaypointMode.STOP)
                .add(x3, y3, start.heading, start.heading, WaypointMode.STOP)
                .add(start.x, start.y, start.heading, start.heading, WaypointMode.STOP);

        PlannedRoute route = planner.plan(start, sequence, start.heading);
        if (route == null || route.isEmpty()) {
            telemetry.addLine("Diamond path planning failed; check field clearance.");
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
