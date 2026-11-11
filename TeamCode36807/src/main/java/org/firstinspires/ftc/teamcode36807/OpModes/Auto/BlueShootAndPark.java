package org.firstinspires.ftc.teamcode36807.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode36807.Libs.*;
import org.firstinspires.ftc.teamcode36807.Robots.BeezyBot;

/**
 * Shoot from BLUE_SHOOTING_POSITION_RIGHT, then navigate to BLUE_LOADING_ZONE.
 * The shooter faces backward; the robot maintains its physical shooting heading
 * during parking. Shooting and navigation are non-blocking.
 * The retreat and parking legs are planned independently using restricted zones.
 */
@Autonomous(name = "BlueShootAndPark")
public class BlueShootAndPark extends OpMode {
    private static final double AUTO_LIMIT_SECONDS = 30.0;
    private static final double SPINUP_SECONDS = 0.5;
    private static final double FEED_SECONDS = 6.0;
    private static final double FLYWHEEL_POWER = 0.5;
    private static final double RETREAT_DISTANCE_INCHES = 12.0;
    private static final double LIFT_FEED = 0.41;
    private static final double LIFT_REST = 0.5;

    private enum State { READY, SPINUP, FEEDING, RETREATING, PARKING, HOLDING, FAILED, FINISHED }

    private BeezyBot robot;
    private MecanumPathFollower follower;
    private PathPlanner planner;
    private PlannedRoute currentRoute;
    private Pose2D parkingPose;
    private State state = State.READY;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private double parkingTime = Double.NaN;

    @Override
    public void init() {
        robot = new BeezyBot(hardwareMap);
        robot.odom.resetPosAndIMU();
        follower = new MecanumPathFollower(
                robot.drive.getWheel(MecanumDrive.Wheel.FRONT_LEFT),
                robot.drive.getWheel(MecanumDrive.Wheel.FRONT_RIGHT),
                robot.drive.getWheel(MecanumDrive.Wheel.BACK_LEFT),
                robot.drive.getWheel(MecanumDrive.Wheel.BACK_RIGHT));
        planner = new PathPlanner(BioBuzzField.FIELD_SIZE,
                robot.size.width, robot.size.length, 0.0, 2.0, 0.0);
        BioBuzzField.configureRestrictedZones(planner);

        robot.flywheel.setPower(0);
        robot.intake.setPower(0);
        robot.servoLift.setPosition(LIFT_REST);
        telemetry.addLine("Place robot at BLUE_SHOOTING_POSITION_RIGHT, shooter toward hive.");
        telemetry.addLine("START: shoot, then navigate to BLUE_LOADING_ZONE.");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        parkingTime = Double.NaN;
        currentRoute = null;

        Pose2D shootingPose = BioBuzzField.getDestination(
                BioBuzzField.Destination.BLUE_SHOOTING_POSITION_RIGHT);
        // BioBuzzField defines the robot's ACTUAL physical heading (+90 deg).
        // Do not negate or rotate it: backward motion then moves toward field -Y.
        double robotHeading = shootingPose.getHeading(AngleUnit.DEGREES);
        Pose2D startPose = shootingPose;
        robot.odom.update();
        robot.odom.setPosition(startPose);

        Pose2D loadingZone = BioBuzzField.getDestination(
                BioBuzzField.Destination.BLUE_LOADING_ZONE);
        parkingPose = Helper.getPose2D(
                Helper.getPoseX(loadingZone), Helper.getPoseY(loadingZone), robotHeading);

        robot.intake.setPower(0);
        robot.servoLift.setPosition(LIFT_REST);
        robot.flywheel.setPower(FLYWHEEL_POWER);
        robot.rgb.blink(RGBIndicator.Color.YELLOW, 250);
        phaseTimer.reset();
        state = State.SPINUP;
    }

    @Override
    public void loop() {
        // Hard deadline applies to shooting as well as parking.
        if (state != State.FINISHED && runtime.seconds() >= AUTO_LIMIT_SECONDS) {
            finishAuto();
        }
        if (state == State.FINISHED) {
            telemetry.addData("State", state);
            telemetry.addData("Elapsed (s)", "%.1f", runtime.seconds());
            telemetry.update();
            return;
        }

        robot.odom.update();
        Pose2D pose = Helper.getPose2D(
                robot.odom.getPosX(DistanceUnit.INCH),
                robot.odom.getPosY(DistanceUnit.INCH),
                robot.odom.getHeading(AngleUnit.DEGREES));

        switch (state) {
            case SPINUP:
                if (phaseTimer.seconds() >= SPINUP_SECONDS) {
                    robot.servoLift.setPosition(LIFT_FEED);
                    phaseTimer.reset();
                    state = State.FEEDING;
                }
                break;

            case FEEDING:
                if (phaseTimer.seconds() >= FEED_SECONDS) {
                    stopShooting();
                    planRetreat(pose);
                }
                break;

            case RETREATING:
                follower.update(pose);
                if (!follower.isFollowing()) {
                    // Plan the rest from the MEASURED pose, not the nominal retreat point.
                    planParking(pose);
                }
                break;

            case PARKING:
                follower.update(pose);
                if (!follower.isFollowing()) {
                    follower.startHolding(parkingPose);
                    parkingTime = runtime.seconds();
                    robot.rgb.green();
                    state = State.HOLDING;
                }
                break;

            case HOLDING:
                follower.holdPosition(pose);
                break;

            case FAILED:
            case READY:
            default:
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Elapsed (s)", "%.1f", runtime.seconds());
        telemetry.addData("Phase (s)", "%.1f", phaseTimer.seconds());
        telemetry.addData("Position (in)", "X %.2f  Y %.2f",
                Helper.getPoseX(pose), Helper.getPoseY(pose));
        telemetry.addData("Heading (deg)", "%.1f", Helper.getPoseHeading(pose));
        telemetry.addData("Parking target (in)", "X %.2f  Y %.2f",
                Helper.getPoseX(parkingPose), Helper.getPoseY(parkingPose));
        telemetry.addData("Parking distance (in)", "%.2f",
                Math.hypot(Helper.getPoseX(parkingPose) - Helper.getPoseX(pose),
                        Helper.getPoseY(parkingPose) - Helper.getPoseY(pose)));
        if (currentRoute != null && !currentRoute.isEmpty()) {
            telemetry.addData("Route segments", currentRoute.segments().size());
            telemetry.addData("Segment index", follower.segmentIndex());
            telemetry.addData("Follower target", "X %.2f  Y %.2f",
                    follower.getTargetX(), follower.getTargetY());
        }
        if (state == State.FAILED) {
            telemetry.addData("Failure reason", planner.getFailureReason());
            telemetry.addData("Failure details", planner.getFailureDetails());
            telemetry.addData("Failed waypoint", planner.getFailedWaypoint());
        }
        telemetry.addData("Parking time (s)", "%.2f", parkingTime);
        telemetry.update();
    }

    /**
     * First stage: back away from the shooting location toward field -Y.
     * This segment is also planned through PathPlanner, so a blocked retreat
     * fails safely instead of driving straight through a restricted zone.
     */
    private void planRetreat(Pose2D pose) {
        double x = Helper.getPoseX(pose);
        double y = Helper.getPoseY(pose);
        double heading = Helper.getPoseHeading(pose);

        PathSequence retreat = new PathSequence()
                .add(x, y - RETREAT_DISTANCE_INCHES,
                        heading, heading, WaypointMode.PASS_THROUGH);

        PlannedRoute route = planner.plan(pose, retreat, heading);
        if (route == null || route.isEmpty()) {
            failPlanning();
            return;
        }
        currentRoute = route;
        follower.start(route);
        robot.rgb.blink(RGBIndicator.Color.YELLOW, 250);
        state = State.RETREATING;
    }

    /**
     * Second stage: let A* / PathPlanner select a safe route around the HIVE.
     * Do NOT force an intermediate waypoint at (startX, parkY): that could
     * take the robot too close to the HIVE supports.
     */
    private void planParking(Pose2D pose) {
        double heading = Helper.getPoseHeading(pose);
        PathSequence destination = new PathSequence()
                .add(Helper.getPoseX(parkingPose), Helper.getPoseY(parkingPose),
                        heading, heading, WaypointMode.STOP);

        PlannedRoute route = planner.plan(pose, destination, heading);
        if (route == null || route.isEmpty()) {
            failPlanning();
            return;
        }
        currentRoute = route;
        follower.start(route);
        robot.rgb.blink(RGBIndicator.Color.YELLOW, 250);
        state = State.PARKING;
    }

    private void failPlanning() {
        follower.stop();
        robot.drive.setPower(0, 0, 0);
        robot.rgb.off();
        state = State.FAILED;
    }

    private void stopShooting() {
        robot.flywheel.setPower(0);
        robot.intake.setPower(0);
        robot.servoLift.setPosition(LIFT_REST);
    }

    private void finishAuto() {
        if (follower != null) follower.cancel();
        stopShooting();
        robot.stop();
        state = State.FINISHED;
    }

    @Override
    public void stop() {
        if (robot != null) finishAuto();
    }
}
