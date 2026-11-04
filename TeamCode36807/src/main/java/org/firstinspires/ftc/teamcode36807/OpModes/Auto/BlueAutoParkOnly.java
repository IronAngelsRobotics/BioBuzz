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
 * Starts at the blue-left shooting position and navigates once to the
 * blue loading/parking zone, avoiding the configured restricted zones.
 * Keeps the starting heading and holds the parking pose on arrival.
 */
@Autonomous(name = "BlueParkOnly")
public class BlueAutoParkOnly extends OpMode {
    private BeezyBot robot;
    private MecanumPathFollower follower;
    private PathPlanner planner;
    private PlannedRoute currentRoute;
    private Pose2D parkingPose;
    private boolean routeAttempted;
    private boolean holding;
    private boolean planningFailed;
    private final ElapsedTime runtime = new ElapsedTime();
    private boolean autoFinished = false;
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
                robot.size.width, robot.size.length, 0.0, 2.0);
        BioBuzzField.configureRestrictedZones(planner);
        telemetry.addLine("Place robot at BLUE_SHOOTING_POSITION_LEFT, heading toward hive.");
        telemetry.addLine("Will navigate to BLUE_LOADING_ZONE when START is pressed.");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();

        routeAttempted = false;
        holding = false;
        planningFailed = false;
        currentRoute = null;

        // Configure the robot's starting position
        Pose2D startPose = BioBuzzField.getDestination(
                BioBuzzField.Destination.BLUE_SHOOTING_POSITION_LEFT);
        robot.odom.update();
        robot.odom.setPosition(startPose);


        Pose2D loadingZone = BioBuzzField.getDestination(
                BioBuzzField.Destination.BLUE_LOADING_ZONE);

        // If loading-zone heading is NaN (unspecified). Preserve the start heading.
        double heading = Double.isNaN(Helper.getPoseHeading(loadingZone))
                ? Helper.getPoseHeading(startPose)
                : Helper.getPoseHeading(loadingZone);

        parkingPose = Helper.getPose2D(
                Helper.getPoseX(loadingZone), Helper.getPoseY(loadingZone),
                heading);

        robot.rgb.green();
        robot.servoLift.setPosition(0.5);
    }

    @Override
    public void loop() {
        // Nothing to do after autonomous finishes.
        if (autoFinished) return;

        if (runtime.seconds() >= 30.0) {
            if (follower != null) follower.stop();
            robot.stop();
            autoFinished = true;
            return;
        }

        robot.odom.update();
        Pose2D pose = Helper.getPose2D(
                robot.odom.getPosX(DistanceUnit.INCH),
                robot.odom.getPosY(DistanceUnit.INCH),
                robot.odom.getHeading(AngleUnit.DEGREES));

        if (!routeAttempted) {
            routeAttempted = true;

            // Current robot position
            double startX = Helper.getPoseX(pose);
            double heading = Helper.getPoseHeading(pose);

            // Final parking position
            double parkX = Helper.getPoseX(parkingPose);
            double parkY = Helper.getPoseY(parkingPose);

            // Intermediate waypoint:
            // Move forward first, keeping the same X coordinate.
            double waypointX = startX;
            double waypointY = parkY;

            // Create an L-shaped route.
            PathSequence sequence = new PathSequence()

                    // Step 1: Move forward to (83.5, 36)
                    .add(
                            waypointX,
                            waypointY,
                            heading,
                            heading,
                            WaypointMode.PASS_THROUGH
                    )

                    // Step 2: Strafe right to (132, 36)
                    .add(
                            parkX,
                            parkY,
                            heading,
                            heading,
                            WaypointMode.STOP
                    );

            currentRoute = planner.plan(
                    pose,
                    sequence,
                    heading
            );
            if (currentRoute == null || currentRoute.isEmpty()) {
                planningFailed = true;
                robot.drive.setPower(0, 0, 0);
                robot.rgb.off();
            } else {
                follower.start(currentRoute);
            }
        }

        if (follower.isFollowing()) {
            follower.update(pose);
            if (!follower.isFollowing()) {
                follower.startHolding(parkingPose);
                holding = true;
                robot.rgb.green();
            }
        } else if (holding) {
            follower.holdPosition(pose);
        }

        if (!follower.isFollowing() && Double.isNaN(parkingTime)) {
            parkingTime = runtime.seconds();
        }

        telemetry.addData("State", planningFailed ? "PLANNING FAILED" :
                follower.isFollowing() ? "Parking" : holding ? "Holding" : "Ready");
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
            telemetry.addData("Follower target", "X %.2f  Y %.2f",
                    follower.getTargetX(), follower.getTargetY());
        }

        if (planningFailed) telemetry.addLine("Check destination and restricted-zone clearance.");

        telemetry.addData("Parking Time", "%.2f seconds", parkingTime);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) follower.stop();
        if (robot != null) robot.stop();
    }
}
