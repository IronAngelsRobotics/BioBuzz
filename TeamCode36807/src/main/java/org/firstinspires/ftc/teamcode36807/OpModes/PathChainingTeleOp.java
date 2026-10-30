
package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode36807.Libs.*;
import org.firstinspires.ftc.teamcode36807.Robots.BeezyBot;

/**
 * Demonstrates a three-segment autonomous-style path in a TeleOp:
 * forward 24 inches, rotate 180 degrees, and return to the start.
 *
 * Y starts the route. X pauses/resumes motor output and freezes diagnostic
 * telemetry. After the route, station keeping holds the starting position
 * at the reversed heading. Press STOP to disable all motor commands.
 *
 * Coordinate contract:
 *   - Pinpoint returns field-relative inches and CCW-positive degrees.
 *   - Planner uses the same axes with a +72 inch X/Y display offset.
 *   - Follower accepts the planner pose; never pass raw Pinpoint coordinates
 *     to the follower when the route was planned with the offset pose.
 */

@TeleOp(name = "Path Chaining")
//@Disabled
public class PathChainingTeleOP extends OpMode {

    private static final double DISTANCE = 24.0;
    private static final double FIELD_SIZE_INCHES = 144.0;
    private static final double PLANNER_ORIGIN_OFFSET_INCHES = FIELD_SIZE_INCHES / 2.0;
    private static final double EXTRA_CLEARANCE_INCHES = 0.0;
    private static final double PLANNER_GRID_INCHES = 2.0;

    // Adjust these to match your robot's dimensions.
    private static final double ROBOT_WIDTH = 17.0;
    private static final double ROBOT_LENGTH = 15.0;

    private BeezyBot robot;
    private MecanumPathFollower follower;
    private PathPlanner planner;

    // Rising-edge button detection prevents held buttons from repeating actions.
    private boolean previousY = false;
    private boolean previousX = false;

    // Pausing preserves the route state but immediately stops all motors (wheels).
    private boolean paused = false;
    private Pose2D pausedPose;
    private int pausedSegment;
    private double pausedTargetX, pausedTargetY;
    private double pausedForward, pausedStrafe;
    private boolean pausedRecovering;

    // Diagnostics to aid troubleshooting.
    // First command calculated while the return segment (index 2) is active.
    private boolean firstReturnCommandCaptured = false;
    private double firstReturnForward, firstReturnStrafe;
    private double firstReturnX, firstReturnY, firstReturnHeading;

    // Fixed snapshots at the start and at each segment transition.
    private Pose2D initialPoseSnapshot;
    private Pose2D beforeRotationSnapshot;
    private Pose2D afterRotationSnapshot;

    // Final hold target: starting X/Y but heading reversed 180 degrees.
    private Pose2D startingPose;
    private boolean finalPositionCorrection = false;

    private PlannedRoute currentRoute;
    private Pose2D originalStartingPose = null;

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

        planner = new PathPlanner(
                FIELD_SIZE_INCHES,
                ROBOT_WIDTH,
                ROBOT_LENGTH,
                EXTRA_CLEARANCE_INCHES,
                PLANNER_GRID_INCHES
        );

        telemetry.addLine("Path chaining ready");
        telemetry.addLine("Press Y to start");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Update Pinpoint exactly once; both follower and telemetry consume
        // the same position snapshot to avoid inconsistent measurements.
        robot.odom.update();

        Pose2D pose = getPose();

        // Reset Pinpoint (0,0) maps to planner (72,72). This is a SOFTWARE
        // origin translation, not a physical field-localization measurement.
        Pose2D plannerPose = new Pose2D(
                pose.x + PLANNER_ORIGIN_OFFSET_INCHES,
                pose.y + PLANNER_ORIGIN_OFFSET_INCHES,
                pose.heading
        );

        // Detect a new Y button press.
        boolean yPressed = gamepad1.y && !previousY;
        previousY = gamepad1.y;

        // X toggles pause/resume. The robot's actual odometry continues to
        // update, but the displayed diagnostic pose is frozen while paused.
        // Resume uses the latest live pose; it does not reset the path.
        boolean xPressed = gamepad1.x && !previousX;
        previousX = gamepad1.x;
        if (xPressed && (follower.isFollowing() || finalPositionCorrection)) {
            paused = !paused;
            if (paused) {
                pausedPose = plannerPose;
                pausedSegment = follower.segmentIndex();
                pausedTargetX = follower.getTargetX();
                pausedTargetY = follower.getTargetY();
                pausedForward = follower.getCommandedForward();
                pausedStrafe = follower.getCommandedStrafe();
                pausedRecovering = follower.isRecovering();
                follower.stop(); // Stops all four motors; does not cancel route.
            }
        }

        // Y starts a new chain only when idle and not paused.
        if (!paused && yPressed && !follower.isFollowing()
                && !finalPositionCorrection) {
            startPathChain(plannerPose);
        }

        // Pause skips both path following and station keeping.
        if (!paused && follower.isFollowing()) {
            // Capture segment-boundary poses only after the follower updates.
            // Segment indices are zero-based: 0=forward, 1=rotate, 2=return.
            int segmentBeforeUpdate = follower.segmentIndex();
            follower.update(plannerPose);
            int segmentAfterUpdate = follower.segmentIndex();
            if (segmentBeforeUpdate == 0 && segmentAfterUpdate >= 1
                    && beforeRotationSnapshot == null) {
                beforeRotationSnapshot = plannerPose;
            }
            if (segmentBeforeUpdate == 1 && segmentAfterUpdate >= 2
                    && afterRotationSnapshot == null) {
                afterRotationSnapshot = plannerPose;
            }
            // This diagnostic may capture a zero command during the 250 ms
            // post-rotation settling interval; it is not necessarily motion.
            if (!firstReturnCommandCaptured && segmentBeforeUpdate == 2) {
                firstReturnCommandCaptured = true;
                firstReturnForward = follower.getCommandedForward();
                firstReturnStrafe = follower.getCommandedStrafe();
                firstReturnX = plannerPose.x;
                firstReturnY = plannerPose.y;
                firstReturnHeading = plannerPose.heading;
            }
            if (!follower.isFollowing() && startingPose != null) {
                follower.startHolding(startingPose);
                finalPositionCorrection = true;
            }
        }

        if (!paused && finalPositionCorrection) {
            follower.holdPosition(plannerPose);
        }

        // Calculate distance from the original target position.
        // This measures accuracy relative to Pinpoint's reported position.
        // It won't reveal accumulated odometry drift relative to the actual field.
        if (startingPose != null) {

            double errorX = startingPose.x - plannerPose.x;
            double errorY = startingPose.y - plannerPose.y;

            double positionError = Math.hypot(errorX, errorY);

            telemetry.addData("Final Error X", "%.3f in", errorX);
            telemetry.addData("Final Error Y", "%.3f in", errorY);
            telemetry.addData("Final Position Error", "%.3f in", positionError);
        }

        Pose2D displayPose = paused ? pausedPose : plannerPose;
        int displaySegment = paused ? pausedSegment : follower.segmentIndex();
        double displayTargetX = paused ? pausedTargetX : follower.getTargetX();
        double displayTargetY = paused ? pausedTargetY : follower.getTargetY();
        double displayForward = paused ? pausedForward : follower.getCommandedForward();
        double displayStrafe = paused ? pausedStrafe : follower.getCommandedStrafe();
        boolean displayRecovering = paused ? pausedRecovering : follower.isRecovering();

        // Single telemetry block: planner and follower use the same coordinates.
        telemetry.addData("Mode", paused ? "PAUSED (snapshot)"
                : finalPositionCorrection ? "Holding"
                : follower.isFollowing() ? "Following" : "Idle");
        telemetry.addLine("X: Pause / Resume    Y: Start when idle");
        telemetry.addData("Segment", displaySegment);
        telemetry.addData("Robot (X,Y)", "(%.2f, %.2f)", displayPose.x, displayPose.y);
        telemetry.addData("Heading (deg)", "%.1f", displayPose.heading);
        telemetry.addData("Target (X,Y)", "(%.2f, %.2f)",
                displayTargetX, displayTargetY);
        telemetry.addData("Forward Command", "%.3f", displayForward);
        telemetry.addData("Strafe Command", "%.3f", displayStrafe);
        telemetry.addData("Recovering", displayRecovering);
        telemetry.addLine("--- POSITION CHECKPOINTS ---");
        addPoseSnapshot("Start", initialPoseSnapshot);
        addPoseSnapshot("Before Rotation", beforeRotationSnapshot);
        addPoseSnapshot("After Rotation", afterRotationSnapshot);
        telemetry.addData("First Return Command Captured", firstReturnCommandCaptured);
        if (firstReturnCommandCaptured) {
            telemetry.addData("First Return Forward", "%.3f", firstReturnForward);
            telemetry.addData("First Return Strafe", "%.3f", firstReturnStrafe);
            telemetry.addData("First Return Pose", "(%.2f, %.2f, %.1f deg)",
                    firstReturnX, firstReturnY, firstReturnHeading);
        }

        if (currentRoute == null) {
            telemetry.addLine("Planned Path: NOT GENERATED");
        } else {
            telemetry.addData("Route Segments", currentRoute.segments().size());
            int i = displaySegment;
            if (i >= 0 && i < currentRoute.segments().size()) {
                PathSegment segment = currentRoute.segments().get(i);
                telemetry.addData("Current Path", "(%.1f, %.1f) -> (%.1f, %.1f)",
                        segment.curve.p0.x, segment.curve.p0.y,
                        segment.curve.p3.x, segment.curve.p3.y);
            }
        }
        telemetry.update();
    }

    private void startPathChain(Pose2D start) {

        // Save the original starting position only once.
        // Subsequent path runs will return to this same position.
        if (originalStartingPose == null)
             originalStartingPose = new Pose2D(start.x, start.y, start.heading);

        // Preserve the original return destination.
        // The robot should face 180 degrees from its original heading.
        startingPose = new Pose2D(
                originalStartingPose.x,
                originalStartingPose.y,
                normalizeHeading(originalStartingPose.heading + 180.0)
        );

        // Reset all diagnostics and station-keeping state for this new run.
        finalPositionCorrection = false;
        firstReturnCommandCaptured = false;
        initialPoseSnapshot = start;
        beforeRotationSnapshot = null;
        afterRotationSnapshot = null;

        double headingRad = Math.toRadians(start.heading);

        // Advance along the initial heading in fixed-field coordinates.
        // This works even if the robot does not start facing field +X.
        double forwardX = start.x + DISTANCE * Math.cos(headingRad);
        double forwardY = start.y + DISTANCE * Math.sin(headingRad);

        // Heading after rotating 180 degrees.
        double reverseHeading = normalizeHeading(start.heading + 180.0);

        // STOP means the robot must settle at each waypoint before moving
        // to the next segment. The second segment is rotation-only.
        PathSequence sequence = new PathSequence()

                // Segment 1: Drive forward 24 inches.
                .add(forwardX, forwardY, start.heading, start.heading, WaypointMode.STOP)

                // Segment 2: Rotate 180 degrees in place.
                .add(forwardX, forwardY, start.heading, reverseHeading, WaypointMode.STOP)

                // Segment 3: Return to the original starting position.
                .add(originalStartingPose.x, originalStartingPose.y, reverseHeading, reverseHeading, WaypointMode.STOP);

        // Convert waypoints into a planned route.
        PlannedRoute route = planner.plan(start, sequence, start.heading);
        currentRoute = route;

        if (route == null || route.isEmpty()) {
            telemetry.addLine("Path planning failed");
            return;
        }

        // Start following the entire route.
        follower.start(route);
    }

    private void addPoseSnapshot(String label, Pose2D pose) {
        if (pose == null) {
            telemetry.addData(label, "Waiting");
        } else {
            telemetry.addData(label, "(%.2f, %.2f, %.1f deg)",
                    pose.x, pose.y, pose.heading);
        }
    }

    /** Read Pinpoint in inches and degrees; no coordinate offset here. */
    private Pose2D getPose() {

        return new Pose2D(
                robot.odom.getPosX(DistanceUnit.INCH),
                robot.odom.getPosY(DistanceUnit.INCH),
                robot.odom.getHeading(AngleUnit.DEGREES)
        );
    }

    /** Normalize a heading into the inclusive [-180, 180] degree range. */
    private double normalizeHeading(double heading) {

        while (heading > 180) heading -= 360;
        while (heading < -180) heading += 360;

        return heading;
    }

    @Override
    public void stop() {

        // FTC STOP must disable both route following and station keeping.
        follower.cancel();
        finalPositionCorrection = false;
        paused = false;
    }
}

