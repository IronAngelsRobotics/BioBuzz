package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode36807.Libs.*;
import org.firstinspires.ftc.teamcode36807.Robots.BeezyBot;

/**
 * S-shaped path test using the existing PathPlanner and MecanumPathFollower.
 * The robot maintains a constant heading as it moves along the path.
 * Y starts the test. Intermediate waypoints use PASS_THROUGH.
 */
@TeleOp(name = "CurvedPath")
public class CurvedPath extends OpMode {
    private static final double ROBOT_WIDTH = 17.0;
    private static final double ROBOT_LENGTH = 15.0;

    private BeezyBot robot;
    private MecanumPathFollower follower;
    private PathPlanner planner;
    private PlannedRoute currentRoute;
    private boolean previousY;
    private boolean holding;
    private Pose2D endingPose;

    @Override
    public void init() {
        robot = new BeezyBot(hardwareMap);
        robot.odom.resetPosAndIMU();
        follower = new MecanumPathFollower(
                robot.drive.getWheel(MecanumDrive.Wheel.FRONT_LEFT),
                robot.drive.getWheel(MecanumDrive.Wheel.FRONT_RIGHT),
                robot.drive.getWheel(MecanumDrive.Wheel.BACK_LEFT),
                robot.drive.getWheel(MecanumDrive.Wheel.BACK_RIGHT));
        planner = new PathPlanner(144.0, ROBOT_WIDTH, ROBOT_LENGTH, 0.0, 2.0);
        telemetry.addLine("CurvedPath ready - press Y to start");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.odom.update();
        Pose2D raw = new Pose2D(
                robot.odom.getPosX(DistanceUnit.INCH),
                robot.odom.getPosY(DistanceUnit.INCH),
                robot.odom.getHeading(AngleUnit.DEGREES));
        Pose2D pose = new Pose2D(raw.x + 72.0, raw.y + 72.0, raw.heading);

        boolean yPressed = gamepad1.y && !previousY;
        previousY = gamepad1.y;
        if (yPressed && !follower.isFollowing() && !holding) {
            startSPath(pose);
        }

        if (follower.isFollowing()) {
            follower.update(pose);
            if (!follower.isFollowing() && endingPose != null) {
                follower.startHolding(endingPose);
                holding = true;
            }
        }
        if (holding) follower.holdPosition(pose);

        telemetry.addData("Mode", holding ? "Holding" : follower.isFollowing() ? "Following" : "Idle");
        telemetry.addData("Segment", follower.segmentIndex());
        telemetry.addData("Position", "(%.2f, %.2f)", pose.x, pose.y);
        telemetry.addData("Heading", "%.1f deg", pose.heading);
        telemetry.addData("Target", "(%.2f, %.2f)", follower.getTargetX(), follower.getTargetY());
        telemetry.addData("Forward / Strafe", "%.3f / %.3f",
                follower.getCommandedForward(), follower.getCommandedStrafe());
        telemetry.addData("Recovering", follower.isRecovering());
        if (endingPose != null) {
            telemetry.addData("End error total", "%.3f in",
                    Math.hypot(pose.x - endingPose.x, pose.y - endingPose.y));
        }
        telemetry.addData("Route segments", currentRoute == null ? 0 : currentRoute.segments().size());
        telemetry.update();
    }

    private void startSPath(Pose2D start) {
        // Coordinates are field-relative; X is forward at heading 0.
        // Use heading-relative offsets so the same test works at any initial heading.
        double a = Math.toRadians(start.heading);
        double fx = Math.cos(a), fy = Math.sin(a);
        double lx = -Math.sin(a), ly = Math.cos(a);

        // Alternating lateral offsets create an S-shaped route.
        // Travel directions at the waypoints shape the Bezier tangents.
        PathSequence sequence = new PathSequence()
                .add(start.x + 12*fx + 10*lx, start.y + 12*fy + 10*ly,
                        start.heading + 35, start.heading, WaypointMode.PASS_THROUGH)
                .add(start.x + 24*fx, start.y + 24*fy,
                        start.heading - 35, start.heading, WaypointMode.PASS_THROUGH)
                .add(start.x + 36*fx - 10*lx, start.y + 36*fy - 10*ly,
                        start.heading - 35, start.heading, WaypointMode.PASS_THROUGH)
                .add(start.x + 48*fx, start.y + 48*fy,
                        start.heading, start.heading, WaypointMode.STOP);

        // The last waypoint is the hold target, not the starting point.
        endingPose = new Pose2D(start.x + 48*fx, start.y + 48*fy, start.heading);
        currentRoute = planner.plan(start, sequence, start.heading);
        if (currentRoute == null || currentRoute.isEmpty()) {
            endingPose = null;
            telemetry.addLine("Path planning failed. Check field clearance.");
            return;
        }
        follower.start(currentRoute);
    }

    @Override
    public void stop() {
        if (follower != null) follower.cancel();
    }
}
