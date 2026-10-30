package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode36807.Libs.*;

/** Example. Match hardware names, motor directions, Pinpoint calibration and axes to your robot. */
@TeleOp(name="Safe Path Sequence Demo")
@Disabled
public class PathingOpMode extends LinearOpMode {
    private GoBildaPinpointDriver pinpoint;
    private MecanumPathFollower follower;
    private PathPlanner planner;
    private double previousX,previousY,velocityX,velocityY;
    private long previousPoseTime;
    private boolean velocityInitialized;
    private double velocityFilterTau=0.06;

    private void updateVelocity(Pose2D pose) {
        long now=System.nanoTime();
        if(!velocityInitialized) {
            previousX=pose.x;previousY=pose.y;previousPoseTime=now;
            velocityX=velocityY=0;velocityInitialized=true;return;
        }
        double dt=(now-previousPoseTime)*1e-9;
        if(dt<=0||dt>0.1) {
            previousX=pose.x;previousY=pose.y;previousPoseTime=now;
            velocityX=velocityY=0;return;
        }
        double mx=(pose.x-previousX)/dt,my=(pose.y-previousY)/dt;
        double alpha=dt/(velocityFilterTau+dt);
        velocityX+=alpha*(mx-velocityX);velocityY+=alpha*(my-velocityY);
        previousX=pose.x;previousY=pose.y;previousPoseTime=now;
    }
    private double startTravelDirection(Pose2D pose,double goalX,double goalY) {
        if(Math.hypot(velocityX,velocityY)>2.0)
            return Math.toDegrees(Math.atan2(velocityY,velocityX));
        // Stationary: depart toward goal rather than assuming facing == travel.
        return Math.toDegrees(Math.atan2(goalY-pose.y,goalX-pose.x));
    }
    private void beginRoute(Pose2D pose) {
        PathSequence requested=new PathSequence()
            .add(24,36,0,0,WaypointMode.PASS_THROUGH)
            .add(72,48,45,90,WaypointMode.PASS_THROUGH)
            .add(96,96,90,180,WaypointMode.STOP);
        PathTarget first=requested.targets().get(0);
        PlannedRoute route=planner.plan(pose,requested,
            startTravelDirection(pose,first.x,first.y));
        if(route==null) {
            follower.cancel();
            telemetry.addLine("NO SAFE ROUTE - check zones/clearance");
        } else follower.start(route);
    }
    @Override public void runOpMode() {
        DcMotorEx fl=hardwareMap.get(DcMotorEx.class,"frontLeft");
        DcMotorEx fr=hardwareMap.get(DcMotorEx.class,"frontRight");
        DcMotorEx bl=hardwareMap.get(DcMotorEx.class,"backLeft");
        DcMotorEx br=hardwareMap.get(DcMotorEx.class,"backRight");
        pinpoint=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        // REQUIRED: configure motor directions and Pinpoint offsets/pod directions
        // to match your actual robot before running. Do not reset pose unexpectedly.
        follower=new MecanumPathFollower(fl,fr,bl,br);
        planner=new PathPlanner(144,18,18,2,3);
        // EXAMPLE ONLY: Replace with actual no-go zones in field coordinates.
        planner.addRestrictedZone(new RestrictedZone(50,50,62,62));
        planner.addRestrictedZone(new RestrictedZone(90,52,102,64));
        planner.addRestrictedZone(new RestrictedZone(52,102,64,114));
        telemetry.addLine("A: plan route. Move sticks: cancel and drive manually.");
        telemetry.update();
        waitForStart();
        if(isStopRequested()) return;
        boolean oldA=false;
        try {
            while(opModeIsActive()) {
                pinpoint.update(); // exactly once per loop
                Pose2D pose=new Pose2D(pinpoint.getPosX(DistanceUnit.INCH),
                    pinpoint.getPosY(DistanceUnit.INCH),pinpoint.getHeading(AngleUnit.DEGREES));
                updateVelocity(pose);
                boolean pressed=gamepad1.a&&!oldA;oldA=gamepad1.a;
                if(pressed) beginRoute(pose);
                double forward=-gamepad1.left_stick_y;
                double strafe=gamepad1.left_stick_x;
                double turn=gamepad1.right_stick_x;
                boolean manual=Math.abs(forward)>0.15||Math.abs(strafe)>0.15||Math.abs(turn)>0.15;
                if(manual) {
                    if(follower.isFollowing()) follower.cancel();
                    follower.driveRobotCentric(forward,strafe,turn);
                } else if(follower.isFollowing()) follower.update(pose);
                else follower.stop();
                telemetry.addData("X",pose.x);telemetry.addData("Y",pose.y);
                telemetry.addData("Heading",pose.heading);
                telemetry.addData("Vx",velocityX);telemetry.addData("Vy",velocityY);
                telemetry.addData("Following",follower.isFollowing());
                telemetry.addData("Segment",follower.segmentIndex());
                telemetry.addData("Recovering",follower.isRecovering());
                telemetry.addData("Distance from path (in)",follower.getDistanceFromPath());
                telemetry.addData("Recovery events",follower.getRecoveryCount());
                telemetry.update();
                idle();
            }
        } finally { follower.stop(); }
    }
}
