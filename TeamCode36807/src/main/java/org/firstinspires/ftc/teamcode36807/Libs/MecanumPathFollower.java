package org.firstinspires.ftc.teamcode36807.Libs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/**
 * Non-blocking mecanum path follower with PD translation, proportional heading,
 * off-path recovery, and a separate PD station-keeping controller.
 *
 * Coordinate convention:
 *   field +X = robot forward at heading 0; field +Y = robot left at heading 0;
 *   heading positive = counterclockwise (Pinpoint convention);
 *   motor command +forward = forward, +strafe = right, +turn = clockwise.
 *
 * The caller MUST update Pinpoint once per OpMode loop and pass the same
 * field-relative pose snapshot to update() or holdPosition(). Coordinates are
 * inches and headings are degrees. This class never calls Pinpoint.update().
 */
public final class MecanumPathFollower {
    private final DcMotorEx fl,fr,bl,br;
    private PlannedRoute route;
    private int index;
    private boolean running;
    private double progress;
    private double prevForward,prevStrafe,prevTurn;
    private long lastTime;
    private boolean recovering;
    private int recoveryStableSamples;
    private double distanceFromPath;
    private int recoveryCount;

    // Recovery hysteresis: enter at 6 in, exit below 3 in for 4 samples.
    public double recoveryEnterDistance = 6.0;
    public double recoveryExitDistance = 3.0;
    public int recoveryExitSamples = 4;
    public double recoveryMaxPower = 0.25;
    public double recoveryLookahead = 3.0;

    // Path translation PD: P acts on position error; D damps measured velocity.
    public double translationKd = 0.01;
    public double translationKp=0.055;
    public double headingKp=0.014; // Motor power per degree; heading correction is P-only.
    public double maxTranslationPower=0.55,maxTurnPower=0.35;
    public double translationAccelRate=1.5,translationDecelRate=2.5;
    public double turnAccelRate=1.0,turnDecelRate=4.0;
    public double minLookahead=4,maxLookahead=10;

    //This determines when the robot considers the path complete.
    public double positionTolerance=1.5;

    public double headingTolerance=4;
    public double passThroughRadius=2.0;

    private double holdX;
    private double holdY;
    private double holdHeading;

    // Previous position measurements
    private double previousPathX;
    private double previousPathY;
    private boolean pathVelocityInitialized = false;

    // holdPositionKp controls how aggressively the robot moves toward its saved position.
    // Higher values make it respond faster to position errors.
    private double holdPositionKp = 0.06;
    private double holdHeadingKp = 0.06;

    // Translation derivative gain.
    // Uses robot velocity to reduce overshoot near the target.
    // Units: motor power per (inch/second).
    private double holdPositionKd = 0.01;

    // holdMaxTranslationPower limits the maximum motor power used for translation.
    private double holdMaxTranslationPower = 0.20;
    private double holdMaxTurnPower = 0.12;

    private double holdPositionTolerance = .25; // inches
    private double holdHeadingTolerance = 2.0;  // degrees

    private boolean holdingPosition = false;

    // Station-keeping heading controller
    private double holdHeadingKd = 0.0015;

    // Angular velocity estimation
    private double previousHoldHeading = 0;
    private double filteredAngularVelocity = 0;

    private boolean headingVelocityInitialized = false;

    // Angular velocity filter time constant
    private double angularVelocityTau = 0.06;

    // ---------------------------------------------------------
    // Station Keeping - Translation Velocity Tracking
    // ---------------------------------------------------------

    // Previous X position reported by Pinpoint (inches)
    private double previousHoldX;

    // Previous Y position reported by Pinpoint (inches)
    private double previousHoldY;

    // Robot's estimated velocity along the field X-axis
    // Units: inches per second
    private double velocityX;

    // Robot's estimated velocity along the field Y-axis
   // Units: inches per second
    private double velocityY;

    // Indicates whether the previous position has been recorded.
    // Prevents incorrect velocity calculations on the first update.
    private boolean holdVelocityInitialized = false;

    private double commandedForward = 0;
    private double commandedStrafe = 0;

    private double targetX;
    private double targetY;

    // Monotonic deadline (System.nanoTime) for the 250 ms post-rotation pause.
    private long rotationSettlingUntil = 0;

    private static final double DEFAULT_DT_SECONDS = 0.02;
    private static final double MAX_VALID_DT_SECONDS = 0.10;
    private static final double HOLD_MIN_MOVEMENT_POWER = 0.06;
    private static final long ROTATION_SETTLE_NANOS = 250_000_000L;

    // ---------------------------------------------------------
    // Translation Hysteresis
    // ---------------------------------------------------------

    // Stop correcting when within 1.5 inches of the target.
    // This determines when hysteresis stops translation corrections
    private double translationStopTolerance = 0.5;

    // Resume correcting if the robot drifts more than 1.0 inch.
    private double translationResumeTolerance = 1.0;

    // Indicates whether translation corrections are suspended.
    private boolean translationHolding = false;

    //=========================================================================

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    public double getCommandedForward() {
        return commandedForward;
    }

    public double getCommandedStrafe() {
        return commandedStrafe;
    }

    public MecanumPathFollower(DcMotorEx fl,DcMotorEx fr,DcMotorEx bl,DcMotorEx br) {
        this.fl=fl; this.fr=fr; this.bl=bl; this.br=br;
    }

    /** Begin following a newly planned route. Does not reset Pinpoint. */
    public void start(PlannedRoute route) {
        holdingPosition = false;
        translationHolding = false;
        rotationSettlingUntil = 0;
        pathVelocityInitialized = false;
        if(route==null||route.isEmpty()) throw new IllegalArgumentException("Empty route");
        this.route=route; index=0; progress=0; running=true;
        prevForward=prevStrafe=prevTurn=0; lastTime=0;
        recovering=false; recoveryStableSamples=0;
        distanceFromPath=0; recoveryCount=0;
    }

    public boolean isFollowing() { return running; }
    public int segmentIndex() { return index; }
    public boolean isRecovering() { return recovering; }
    public double getDistanceFromPath() { return distanceFromPath; }
    public int getRecoveryCount() { return recoveryCount; }

    /** Cancel path following and station keeping; immediately stop motors. */
    public void cancel() {
        running = false;
        holdingPosition = false;
        route = null;
        recovering = false;
        rotationSettlingUntil = 0;
        stop();
    }

    /** Stop motors without cancelling the route (used by the pause feature). */
    public void stop() {
        fl.setPower(0);fr.setPower(0);bl.setPower(0);br.setPower(0);
        prevForward=prevStrafe=prevTurn=0;
    }

    private double updateAngularVelocity(Pose2D pose, double dt) {
        if (!headingVelocityInitialized) {
            previousHoldHeading = pose.heading;
            filteredAngularVelocity = 0;
            headingVelocityInitialized = true;
            return 0;
        }

        // Change in heading, accounting for wraparound
        double deltaHeading = angleError(pose.heading, previousHoldHeading);

        // Angular velocity in degrees/second
        double measuredAngularVelocity = deltaHeading / dt;

        // Time-based low-pass filter
        double alpha = dt / (angularVelocityTau + dt);

        filteredAngularVelocity += alpha * (measuredAngularVelocity - filteredAngularVelocity);

        previousHoldHeading = pose.heading;

        return filteredAngularVelocity;
    }

    public double getHoldErrorX(Pose2D pose) {
        return holdX - pose.x;
    }

    public double getHoldErrorY(Pose2D pose) {
        return holdY - pose.y;
    }

    public double getHoldHeadingError(Pose2D pose) {
        return angleError(holdHeading, pose.heading);
    }

    /**
     * Calculates the robot's X/Y velocity using Pinpoint position changes.
     *
     * @param pose Current robot position from Pinpoint.
     * @param dt   Time elapsed since the previous update, in seconds.
     *
     * Velocity units: inches per second.
     */
    private void updateHoldVelocity(Pose2D pose, double dt) {

        // Skip invalid intervals rather than dividing by zero.
        if (dt <= 0) {
            return;
        }

        // Initialize tracking on the first update.
        if (!holdVelocityInitialized) {
            previousHoldX = pose.x;
            previousHoldY = pose.y;

            velocityX = 0;
            velocityY = 0;

            holdVelocityInitialized = true;
            return;
        }

        // Calculate velocity from changes in position.
        velocityX = (pose.x - previousHoldX) / dt;
        velocityY = (pose.y - previousHoldY) / dt;

        // Save the current position for the next update.
        previousHoldX = pose.x;
        previousHoldY = pose.y;
    }

    /** Begin active position hold at a fixed field pose (inches, degrees). */
    public void startHolding(Pose2D pose) {

        // Cancel any active path
        cancel();

        holdX = pose.x;
        holdY = pose.y;
        holdHeading = pose.heading;

        // Reset previous motor commands
        prevForward = 0;
        prevStrafe = 0;
        prevTurn = 0;

        holdingPosition = true;
        lastTime = 0;
        previousHoldHeading = pose.heading;
        filteredAngularVelocity = 0;
        headingVelocityInitialized = false;

        // Initialize position tracking using the current Pinpoint position.
        previousHoldX = pose.x;
        previousHoldY = pose.y;

        // Reset estimated velocity when station keeping starts.
        velocityX = 0;
        velocityY = 0;

        // The first holdPosition() update will initialize velocity tracking.
        holdVelocityInitialized = false;

        // Immediately stop motors
        stop();
    }

    public boolean isHoldingPosition() {
        return holdingPosition;
    }

    /**
     * Rotate a fixed-field vector into robot commands. Note the negative Y
     * sign: positive field Y is LEFT, but positive robot strafe is RIGHT.
     */
    private double[] fieldToRobot(double fieldX, double fieldY, double headingDegrees) {
        double heading = Math.toRadians(headingDegrees);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double forward = fieldX * cos + fieldY * sin;
        double strafe = fieldX * sin - fieldY * cos;

        return new double[]{forward, strafe};
    }

    /** Run one non-blocking PD station-keeping update. */
    public void holdPosition(Pose2D pose) {

        if (!holdingPosition) {
            return;
        }

        // ---------------------------------------------------------
        // Calculate elapsed time
        // ---------------------------------------------------------

        long now = System.nanoTime();

        double dt = lastTime == 0 ? DEFAULT_DT_SECONDS : (now - lastTime) * 1e-9;
        lastTime = now;

        // Sanitize BEFORE estimating velocity: a delayed loop must not
        // produce a huge derivative spike or an invalid divide.
        if (dt <= 0 || dt > MAX_VALID_DT_SECONDS) {
            dt = DEFAULT_DT_SECONDS;
            holdVelocityInitialized = false;
            headingVelocityInitialized = false;
        }
        updateHoldVelocity(pose, dt);


        // ---------------------------------------------------------
        // Position error in field coordinates
        // ---------------------------------------------------------

        double errorX = holdX - pose.x;
        double errorY = holdY - pose.y;

        double distanceError =
                Math.hypot(errorX, errorY);


        // ---------------------------------------------------------
        // Convert field error to robot coordinates
        // ---------------------------------------------------------

        double[] movement = fieldToRobot(errorX, errorY, pose.heading);
        double forward = movement[0];
        double strafe = movement[1];


        // ---------------------------------------------------------
        // Translation proportional controller
        // ---------------------------------------------------------

        forward *= holdPositionKp;
        strafe *= holdPositionKp;

        // Convert measured field velocity into robot-relative velocity.
        double headingRadians = Math.toRadians(pose.heading);

        double forwardVelocity = velocityX * Math.cos(headingRadians) + velocityY * Math.sin(headingRadians);
        double strafeVelocity = velocityX * Math.sin(headingRadians) - velocityY * Math.cos(headingRadians);

        // Apply derivative damping to reduce overshoot.
        forward -= holdPositionKd * forwardVelocity;
        strafe -= holdPositionKd * strafeVelocity;

        // Ignore tiny position errors
        if (distanceError < holdPositionTolerance) {
            forward = 0;
            strafe = 0;
        }


        // ---------------------------------------------------------
        // Limit translation power
        // ---------------------------------------------------------

        double magnitude = Math.hypot(forward, strafe);

        if (magnitude > holdMaxTranslationPower) {
            double scale = holdMaxTranslationPower / magnitude;
            forward *= scale;
            strafe *= scale;
        }


        // Heading PD: positive Pinpoint error (CCW) needs negative motor
        // turn because the mecanum mixer uses clockwise-positive turn.

        double headingError = angleError(holdHeading, pose.heading);

        // Estimate angular velocity in degrees per second.
        double angularVelocity = updateAngularVelocity(pose, dt);

        // Proportional correction.
        double proportional = -holdHeadingKp * headingError;

        // Derivative damping (positive measured CCW velocity opposes CW command).
        double derivative = holdHeadingKd * angularVelocity;

        // Combine P and D.
        double turn = proportional + derivative;

        // Limit rotation power.
        turn = clip(turn, -holdMaxTurnPower, holdMaxTurnPower);

        // Suppress tiny corrections only after angular motion settles.
        if (Math.abs(headingError) < holdHeadingTolerance && Math.abs(angularVelocity) < 5.0) {
            turn = 0;
        }


        // Static friction compensation must occur BEFORE slew limiting;
        // otherwise the calculated ramp is bypassed by the motor output.
        if (Math.abs(forward) > 0 && Math.abs(forward) < HOLD_MIN_MOVEMENT_POWER) {
            forward = Math.copySign(HOLD_MIN_MOVEMENT_POWER, forward);
        }
        if (Math.abs(strafe) > 0 && Math.abs(strafe) < HOLD_MIN_MOVEMENT_POWER) {
            strafe = Math.copySign(HOLD_MIN_MOVEMENT_POWER, strafe);
        }

        // Slew-limit the commands actually sent to the drivetrain.
        double[] translation = translationSlew(forward, strafe, dt);

        turn = slew(prevTurn, turn, turnAccelRate, turnDecelRate, dt);
        prevForward = translation[0];
        prevStrafe = translation[1];
        prevTurn = turn;

        // Previously, this method calculated slew-limited values but drove
        // the unsmoothed forward/strafe values. Use the ramped values instead.
        driveRobotCentric(prevForward, prevStrafe, prevTurn);
    }

    /**
     * Returns true when the robot reaches the target position
     * and heading within the configured tolerances.
     */
    public boolean isAtTarget(Pose2D pose) {

        // Calculate distance from the saved target position.
        double distanceError = Math.hypot(holdX - pose.x, holdY - pose.y);

        // Calculate the shortest heading error.
        double headingError = holdHeading - pose.heading;

        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;

        // Both position and heading must be within tolerance.
        return distanceError < holdPositionTolerance && Math.abs(headingError) < holdHeadingTolerance;
    }

    /**
     * Sets a target position and heading for autonomous movement.
     *
     * @param x Target X coordinate in inches
     * @param y Target Y coordinate in inches
     * @param heading Target heading in degrees
     */
    public void setTargetPose(double x, double y, double heading) {
        Pose2D targetPose = new Pose2D(x, y, heading);
        startHolding(targetPose);
    }

    public void stopHolding() {
        holdingPosition = false;
        stop();
    }

    private static double clip(double v,double min,double max) { return Range.clip(v,min,max); }

    private static double angleError(double target,double current) {
        double d=(target-current)%360;
        if(d>180)d-=360; if(d< -180)d+=360;
        return d;
    }

    // Find the nearest point on the WHOLE current curve. This permits
    // recovery after the robot is physically pushed backward.
    private double closestAnywhere(BezierPath curve, Pose2D pose) {
        double bestT=0, bestD=Double.POSITIVE_INFINITY;
        for(int i=0;i<=200;i++) {
            double t=i/200.0;
            BezierPath.Point p=curve.getPoint(t);
            double d=Math.hypot(p.x-pose.x,p.y-pose.y);
            if(d<bestD) { bestD=d; bestT=t; }
        }
        distanceFromPath=bestD;
        return bestT;
    }

    /**
     * Normal tracking never moves progress backward. Recovery deliberately
     * re-acquires the closest point so the robot can recover after a push.
     */
    private double nearestT(BezierPath curve,Pose2D pose) {
        double closest=closestAnywhere(curve,pose);
        if(!recovering && distanceFromPath>=recoveryEnterDistance) {
            recovering=true;
            recoveryStableSamples=0;
            recoveryCount++;
        }
        if(recovering) {
            // Reacquire progress even if pushed behind the previous progress.
            progress=closest;
            if(distanceFromPath<=recoveryExitDistance) recoveryStableSamples++;
            else recoveryStableSamples=0;
            if(recoveryStableSamples>=Math.max(1,recoveryExitSamples)) {
                recovering=false;
                recoveryStableSamples=0;
            }
        } else {
            // Prevent noise from walking progress backward in normal operation.
            // Do not advance merely because the closest point is far ahead
            // after an unusual pose jump: the recovery threshold catches that.
            progress=Math.max(progress,closest);
        }
        return progress;
    }

    /** Approximate arc-length lookahead by sampling the Bézier curve. */
    private double lookaheadT(BezierPath curve,double from,double inches) {
        BezierPath.Point previous=curve.getPoint(from);
        double traveled=0;
        for(double t=from+0.005;t<=1.005;t+=0.005) {
            double tt=Math.min(1,t);
            BezierPath.Point p=curve.getPoint(tt);
            traveled+=Math.hypot(p.x-previous.x,p.y-previous.y);
            if(traveled>=inches||tt>=1) return tt;
            previous=p;
        }
        return 1;
    }

    /**
     * Determines whether translation corrections should be suspended.
     *
     * Hysteresis prevents the robot from repeatedly correcting
     * small position errors near its destination.
     *
     * When approaching the target:
     *   - Stop correcting within 0.5 inches.
     *
     * When holding:
     *   - Resume correcting only after drifting beyond 1.0 inch.
     *
     * @param errorX X position error in inches
     * @param errorY Y position error in inches
     * @return true if translation corrections should be suspended
     */
    private boolean shouldHoldTranslation(double errorX, double errorY) {

        double distanceError = Math.hypot(errorX, errorY);

        if (translationHolding) {

            // Resume translation only after sufficient drift.
            if (distanceError > translationResumeTolerance) {
                translationHolding = false;
            }

        } else {

            // Stop translation once sufficiently close.
            if (distanceError <= translationStopTolerance) {
                translationHolding = true;
            }
        }

        return translationHolding;
    }

    /**
     * Limits how quickly one motor command can change between
     * control-loop updates.
     *
     * The acceleration and deceleration rates are expressed as
     * normalized motor-power changes per second.
     *
     * Example:
     *   acceleration = 1.0
     *   dt = 0.02 seconds
     *
     * The command can change by at most 0.02 per update.
     *
     * @param old Previous motor command
     * @param target Desired motor command
     * @param accel Maximum acceleration rate
     * @param decel Maximum deceleration rate
     * @param dt Time elapsed since the previous update (seconds)
     * @return Rate-limited motor command
     */
    private static double slew(double old, double target, double accel, double decel, double dt) {

        // Select acceleration or deceleration based on
        // whether the requested command magnitude is increasing.
        double rate = Math.abs(target) > Math.abs(old) ? accel : decel;

        // Calculate the maximum permitted change this update.
        double maxChange = rate * dt;

        // Move toward the target without exceeding the limit.
        return old + clip(target - old, -maxChange, maxChange);
    }

    /**
     * Limits how quickly the robot's translation command can change.
     *
     * Forward and strafe are treated as a 2D vector rather than
     * independently limiting each axis.
     *
     * This preserves the direction of the command change while
     * controlling acceleration and deceleration.
     *
     * @param forward Desired robot-forward command
     * @param strafe Desired robot-right command
     * @param dt Elapsed time since the previous update (seconds)
     * @return Array containing the limited forward and strafe commands
     */
    private double[] translationSlew(double forward, double strafe, double dt) {

        // Calculate previous and requested translation magnitudes.
        double oldMagnitude = Math.hypot(prevForward, prevStrafe);

        double newMagnitude = Math.hypot(forward, strafe);

        // A negative dot product indicates that the requested
        // direction differs from the previous direction by
        // more than 90 degrees.
        boolean reversing = prevForward * forward + prevStrafe * strafe < 0;

        // Use acceleration when increasing speed normally.
        // Use deceleration when slowing down or reversing.
        double rate = (!reversing && newMagnitude > oldMagnitude) ? translationAccelRate : translationDecelRate;

        // Calculate the requested change in each direction.
        double deltaForward = forward - prevForward;
        double deltaStrafe = strafe - prevStrafe;

        // Calculate the magnitude of the requested change.
        double requestedChange = Math.hypot(deltaForward, deltaStrafe);

        // Maximum permitted command change during this update.
        double allowedChange = rate * dt;

        // If the requested change exceeds the limit,
        // scale both components proportionally.
        if (requestedChange > allowedChange && requestedChange > 0) {

            double scale = allowedChange / requestedChange;
            deltaForward *= scale;
            deltaStrafe *= scale;
        }

        // Return the updated translation commands.
        return new double[]{prevForward + deltaForward, prevStrafe + deltaStrafe};
    }

    /**
     * Mix robot-relative commands into wheel power and normalize to [-1, 1].
     * The motor direction configuration must make these signs physically true.
     */
    public void driveRobotCentric(double forward,double strafe,double turn) {
        double a=forward+strafe+turn,b=forward-strafe-turn;
        double c=forward-strafe+turn,d=forward+strafe-turn;
        double norm=Math.max(1,Math.max(Math.max(Math.abs(a),Math.abs(b)),
                                     Math.max(Math.abs(c),Math.abs(d))));
        fl.setPower(a/norm);fr.setPower(b/norm);bl.setPower(c/norm);br.setPower(d/norm);
    }

    // Prepare a newly selected segment using the latest pose snapshot.
    // Do not carry velocity, slew history, or recovery state across segments.
    private void enterSegment(int newIndex, Pose2D pose) {
        index = newIndex;
        prevForward = 0;
        prevStrafe = 0;
        prevTurn = 0;
        commandedForward = 0;
        commandedStrafe = 0;
        pathVelocityInitialized = false;
        previousPathX = pose.x;
        previousPathY = pose.y;
        recovering = false;
        recoveryStableSamples = 0;
        distanceFromPath = 0;
        lastTime = 0;
        translationHolding = false;

        PathSegment next = route.segments().get(index);
        progress = next.isRotationOnly() ? 0 : closestAnywhere(next.curve, pose);
        targetX = pose.x;
        targetY = pose.y;
    }

    /** Advance the active route by one loop using the supplied Pinpoint pose. */
    public void update(Pose2D pose) {
        if(!running) return;
        long now=System.nanoTime();
        double dt=lastTime==0?DEFAULT_DT_SECONDS:(now-lastTime)*1e-9;
        lastTime=now;
        if(dt<=0||dt>MAX_VALID_DT_SECONDS) {
            dt=DEFAULT_DT_SECONDS;
            pathVelocityInitialized=false;
        }
        PathSegment segment=route.segments().get(index);
        BezierPath curve=segment.curve;

        // Allow the robot to settle after a rotation.
        if (System.nanoTime() < rotationSettlingUntil) {
            stop();
            return;
        }

        // Handle rotation-only path segments.
        if (segment.isRotationOnly()) {

            double headingError =
                    angleError(segment.endHeading, pose.heading);

            // Positive motor turn is clockwise;
            // positive Pinpoint heading is counterclockwise.
            double turn = clip(
                    -headingKp * headingError,
                    -maxTurnPower,
                    maxTurnPower
            );

            // Stop translating while rotating.
            driveRobotCentric(0, 0, turn);

            // Check whether rotation is complete.
            if (Math.abs(headingError) <= headingTolerance) {
                stop();

                if (index == route.segments().size() - 1) {
                    running = false;
                } else {
                    enterSegment(index + 1, pose);
                    // Wait 250 ms before beginning the next segment.
                    rotationSettlingUntil =
                            System.nanoTime() + ROTATION_SETTLE_NANOS;
                }
            }

            return;
        }

        double t=nearestT(curve,pose);
        BezierPath.Point end=curve.p3;
        double distance=Math.hypot(end.x-pose.x,end.y-pose.y);
        double headingErr=angleError(segment.endHeading,pose.heading);
        boolean finalSegment=index==route.segments().size()-1;
        // PASS_THROUGH: require near-end progress AND physical proximity.
        double transitionRadius=Math.min(passThroughRadius,Math.max(0.5,curve.length()*0.25));
        boolean passed=(!recovering&&t>=0.90&&distance<=transitionRadius);
        boolean arrived=distance<=positionTolerance&&Math.abs(headingErr)<=headingTolerance;
        if(segment.mode==WaypointMode.PASS_THROUGH && !finalSegment && passed) {
            enterSegment(index + 1, pose);
            segment=route.segments().get(index);curve=segment.curve;
            // enterSegment() already initialized progress from this pose.
            t=progress;
            end=curve.p3;
            distance=Math.hypot(end.x-pose.x,end.y-pose.y);
            headingErr=angleError(segment.endHeading,pose.heading);
            finalSegment=index==route.segments().size()-1;
            if (segment.isRotationOnly()) {
                stop();
                return; // Next update handles rotation-only segment.
            }
        } else if((segment.mode==WaypointMode.STOP || finalSegment) && arrived) {
            stop();
            if(finalSegment) running=false;
            else { enterSegment(index + 1, pose); }
            return;
        }
        double speedRatio=clip(Math.hypot(prevForward,prevStrafe)/maxTranslationPower,0,1);
        double lookahead=recovering ? recoveryLookahead :
                minLookahead+(maxLookahead-minLookahead)*speedRatio;
        double lookT=lookaheadT(curve,t,lookahead);
        BezierPath.Point aim=curve.getPoint(lookT);

        boolean stopping=segment.mode==WaypointMode.STOP || finalSegment;
        if(stopping && distance<6) aim=end;

        // Update telemetry target to reflect the actual aiming point.
        targetX = aim.x;
        targetY = aim.y;

        double targetHeading=segment.startHeading+
                angleError(segment.endHeading,segment.startHeading)*lookT;
        if(stopping && distance<6) targetHeading=segment.endHeading;
        double ex=aim.x-pose.x,ey=aim.y-pose.y;
        double theta=Math.toRadians(pose.heading);
        // Convert field-relative position errors to robot-relative movement.
        // At heading 0°, robot forward is +X and robot left is +Y.
        double forward = ex * Math.cos(theta) + ey * Math.sin(theta);
        double strafe = ex * Math.sin(theta) - ey * Math.cos(theta);
        // Estimate field-relative velocity.
        double velocityX = 0;
        double velocityY = 0;

        if (pathVelocityInitialized) {
            velocityX = (pose.x - previousPathX) / dt;
            velocityY = (pose.y - previousPathY) / dt;
        }

        previousPathX = pose.x;
        previousPathY = pose.y;
        pathVelocityInitialized = true;

        // Convert velocity into robot-relative coordinates.
        double forwardVelocity =
                velocityX * Math.cos(theta)
                        + velocityY * Math.sin(theta);

        double strafeVelocity =
                velocityX * Math.sin(theta)
                        - velocityY * Math.cos(theta);

        // PD translation controller: proportional position error minus
        // derivative damping based on measured field velocity.
        forward = translationKp * forward - translationKd * forwardVelocity;
        strafe = translationKp * strafe - translationKd * strafeVelocity;
        double magnitude=Math.hypot(forward,strafe);
        double maxPower=maxTranslationPower;
        if(stopping&&distance<12) maxPower*=clip(distance/12.0,0.18,1);
        if(recovering) maxPower=Math.min(maxPower,recoveryMaxPower);
        if(magnitude>maxPower&&magnitude>0) {forward*=maxPower/magnitude;strafe*=maxPower/magnitude;}

        /*
        Your Pinpoint heading increases counterclockwise, but positive turn in your
        mecanum drive rotates clockwise. The negative sign makes the path follower
        correct its heading in the proper direction.
         */
        double turn = clip(
                -headingKp * angleError(targetHeading, pose.heading),
                -maxTurnPower,
                maxTurnPower
        );

        // ---------------------------------------------------------
// Final Approach Translation Hysteresis
// ---------------------------------------------------------

// Stop correcting translation when close to the final target.
// Resume only if the robot moves outside the resume tolerance.
        boolean suspendTranslation = false;

        if (finalSegment) {

            // Use the existing position tolerance as the stop threshold.
            // Resume only if the robot drifts farther than 2 inches.
            if (translationHolding) {
                if (distance > 2.0) {
                    translationHolding = false;
                }
            } else {
                if (distance <= positionTolerance) {
                    translationHolding = true;
                }
            }

            suspendTranslation = translationHolding;
        }

        // Preserve normal slew limiting during path following.
        double[] v = translationSlew(forward, strafe, dt);

        // When inside the final target tolerance, suppress translation.
        // Reset slew history so old commands cannot restart movement.
        if (suspendTranslation) {
            v[0] = 0.0;
            v[1] = 0.0;
        }

        // Continue normal heading control.
        turn = slew(prevTurn, turn, turnAccelRate, turnDecelRate, dt);

        // Save final commands.
        prevForward = v[0];
        prevStrafe = v[1];
        prevTurn = turn;

        commandedForward = v[0];
        commandedStrafe = v[1];

        driveRobotCentric(v[0], v[1], turn);
    }
}
