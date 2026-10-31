package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode36807.Libs.*;
import org.firstinspires.ftc.teamcode36807.Robots.BeezyBot;

/**
 * Navigate around a stationary 9.125 x 6.125 inch box placed 24 inches
 * ahead of the robot's starting center, with its wide face toward the robot.
 *
 * Important: This is MAP-BASED avoidance, not sensor-based detection.
 * Place the box accurately and clear the surrounding area before pressing Y.
 */
@TeleOp(name = "ObstaclePath")
public class ObstaclePath extends OpMode {
    private static final double FIELD_SIZE = 144.0;
    private static final double FIELD_ORIGIN = 72.0;
    private static final double ROBOT_WIDTH = 17.0;
    private static final double ROBOT_LENGTH = 15.0;
    private static final double BOX_WIDTH = 9.125; // perpendicular to forward
    private static final double BOX_DEPTH = 6.125; // along forward
    private static final double BOX_DISTANCE = 24.0;
    private static final double GOAL_DISTANCE = 48.0;
    private static final double EXTRA_CLEARANCE = 2.0;

    private BeezyBot robot;
    private MecanumPathFollower follower;
    private PathPlanner planner;
    private PlannedRoute route;
    private Pose2D goal;
    private boolean previousY;
    private boolean holding;
    private boolean attempted;

    @Override public void init() {
        robot = new BeezyBot(hardwareMap);
        robot.odom.resetPosAndIMU();
        follower = new MecanumPathFollower(
                robot.drive.getWheel(MecanumDrive.Wheel.FRONT_LEFT),
                robot.drive.getWheel(MecanumDrive.Wheel.FRONT_RIGHT),
                robot.drive.getWheel(MecanumDrive.Wheel.BACK_LEFT),
                robot.drive.getWheel(MecanumDrive.Wheel.BACK_RIGHT));
        planner = new PathPlanner(FIELD_SIZE, ROBOT_WIDTH, ROBOT_LENGTH,
                EXTRA_CLEARANCE, 2.0);
        telemetry.addLine("ObstaclePath ready. Position box 24 in ahead.");
        telemetry.addLine("Box wide face toward robot. Press Y once.");
        telemetry.update();
    }

    @Override public void loop() {
        robot.odom.update();
        Pose2D pose = new Pose2D(
                robot.odom.getPosX(DistanceUnit.INCH) + FIELD_ORIGIN,
                robot.odom.getPosY(DistanceUnit.INCH) + FIELD_ORIGIN,
                robot.odom.getHeading(AngleUnit.DEGREES));
        boolean yPressed = gamepad1.y && !previousY;
        previousY = gamepad1.y;
        if (yPressed && !attempted) {
            attempted = true;
            startObstacleRoute(pose);
        }
        if (follower.isFollowing()) {
            follower.update(pose);
            if (!follower.isFollowing() && goal != null) {
                follower.startHolding(goal);
                holding = true;
            }
        }
        if (holding) follower.holdPosition(pose);

        telemetry.addData("State", follower.isFollowing() ? "Following" :
                holding ? "Holding" : attempted ? "Planning failed" : "Ready");
        telemetry.addData("Position", "X %.2f Y %.2f", pose.x, pose.y);
        telemetry.addData("Heading", "%.1f", pose.heading);
        telemetry.addData("Segment", follower.segmentIndex());
        if (route != null) telemetry.addData("Route segments", route.segments().size());
        if (goal != null) {
            telemetry.addData("Goal error X", "%.3f", goal.x - pose.x);
            telemetry.addData("Goal error Y", "%.3f", goal.y - pose.y);
            telemetry.addData("Goal error total (in)", "%.3f",
                    Math.hypot(goal.x - pose.x, goal.y - pose.y));
        }
        telemetry.update();
    }

    private void startObstacleRoute(Pose2D start) {
        double a = Math.toRadians(start.heading);
        double fx = Math.cos(a), fy = Math.sin(a);
        double lx = -Math.sin(a), ly = Math.cos(a);
        double cx = start.x + BOX_DISTANCE * fx;
        double cy = start.y + BOX_DISTANCE * fy;

        // Axis-aligned bounding rectangle of the box in field coordinates.
        // For a rotated robot/box, the bounding rectangle is conservative.
        double halfX = Math.abs(lx) * BOX_WIDTH / 2.0
                + Math.abs(fx) * BOX_DEPTH / 2.0;
        double halfY = Math.abs(ly) * BOX_WIDTH / 2.0
                + Math.abs(fy) * BOX_DEPTH / 2.0;

        planner.clearRestrictedZones();
        // Assumes RestrictedZone constructor order: minX, minY, maxX, maxY.
        planner.addRestrictedZone(new RestrictedZone(
                cx - halfX, cy - halfY, cx + halfX, cy + halfY));

        goal = new Pose2D(start.x + GOAL_DISTANCE * fx,
                start.y + GOAL_DISTANCE * fy, start.heading);
        PathSequence sequence = new PathSequence()
                .add(goal.x, goal.y, start.heading, start.heading,
                        WaypointMode.STOP);
        route = planner.plan(start, sequence, start.heading);
        if (route == null || route.isEmpty()) {
            goal = null;
            follower.stop();
            return;
        }
        holding = false;
        follower.start(route);
    }

    @Override public void stop() {
        if (follower != null) follower.stop();
    }
}
