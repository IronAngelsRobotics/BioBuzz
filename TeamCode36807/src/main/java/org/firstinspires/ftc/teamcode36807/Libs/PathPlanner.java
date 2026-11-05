package org.firstinspires.ftc.teamcode36807.Libs;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Static obstacle planner. Returns null when no collision-free route exists. */
public final class PathPlanner {
    private final List<RestrictedZone> zones = new ArrayList<>();
    private final double fieldSize, obstacleClearance, grid;
    private final double robotWidth, robotLength, wallMargin;

    /** Diagnostic status of the most recent plan() call. */
    public enum FailureReason {
        NONE, EMPTY_SEQUENCE, HEADING_CHANGE_NOT_SUPPORTED,
        START_OUT_OF_BOUNDS, GOAL_OUT_OF_BOUNDS,
        START_BLOCKED, GOAL_BLOCKED, NO_PATH_FOUND,
        UNSAFE_CURVE, EMPTY_ROUTE
    }

    private FailureReason failureReason = FailureReason.NONE;
    private String failureDetails = "";
    // 1-based index of the requested waypoint; 0 denotes the initial start.
    // -1 means not applicable.
    private int failedWaypoint = -1;

    public FailureReason getFailureReason() { return failureReason; }
    public String getFailureDetails() { return failureDetails; }
    public int getFailedWaypoint() { return failedWaypoint; }

    private void clearFailure() {
        failureReason = FailureReason.NONE;
        failureDetails = "";
        failedWaypoint = -1;
    }

    private void setFailure(FailureReason reason, String details, int waypoint) {
        failureReason = reason;
        failureDetails = details;
        failedWaypoint = waypoint;
    }

    private boolean validatePosition(AStarPathfinder astar, double x, double y,
                                     boolean start, int waypoint) {
        String label = start ? "Start" : "Goal";
        if (!astar.isWithinBounds(x, y)) {
            setFailure(start ? FailureReason.START_OUT_OF_BOUNDS : FailureReason.GOAL_OUT_OF_BOUNDS,
                    String.format(Locale.US,
                            "%s (%.2f, %.2f) outside bounds; allowed X %.2f..%.2f, Y %.2f..%.2f",
                            label, x, y, astar.getMinX(), astar.getMaxX(),
                            astar.getMinY(), astar.getMaxY()), waypoint);
            return false;
        }
        if (astar.isBlocked(x, y)) {
            setFailure(start ? FailureReason.START_BLOCKED : FailureReason.GOAL_BLOCKED,
                    String.format(Locale.US, "%s (%.2f, %.2f) inside expanded restricted zone",
                            label, x, y), waypoint);
            return false;
        }
        return true;
    }

    /** Existing constructor: preserves old behavior for other OpModes. */
    public PathPlanner(double fieldSize, double robotWidth, double robotLength,
                       double extraMargin, double grid) {
        this(fieldSize, robotWidth, robotLength, extraMargin, grid, -1.0);
    }

    /**
     * wallMargin >= 0: use heading-dependent rectangular wall footprint.
     * wallMargin = 0: permit physical contact, but never plan the center outside.
     * wallMargin < 0: legacy circular clearance at walls (backward compatibility).
     * Obstacle clearance is deliberately unchanged.
     */
    public PathPlanner(double fieldSize, double robotWidth, double robotLength,
                       double extraMargin, double grid, double wallMargin) {
        if (fieldSize <= 0 || robotWidth <= 0 || robotLength <= 0 ||
                extraMargin < 0 || grid <= 0)
            throw new IllegalArgumentException("Invalid field/robot/planner dimensions");
        this.fieldSize = fieldSize;
        this.grid = grid;
        this.robotWidth = robotWidth;
        this.robotLength = robotLength;
        this.wallMargin = wallMargin;
        this.obstacleClearance = Math.hypot(robotWidth / 2.0, robotLength / 2.0) + extraMargin;
    }

    public void addRestrictedZone(RestrictedZone zone) {
        zones.add(zone.expanded(obstacleClearance));
    }
    public void clearRestrictedZones() { zones.clear(); }

    private boolean safeCurve(BezierPath curve, AStarPathfinder astar) {
        BezierPath.Point last = curve.p0;
        for (int i = 1; i <= 150; i++) {
            BezierPath.Point next = curve.getPoint(i / 150.0);
            if (!astar.clearLine(last, next)) return false;
            last = next;
        }
        return true;
    }

    private BezierPath candidate(BezierPath.Point a, BezierPath.Point b,
                                 double startDirection, double endDirection) {
        double d = Math.hypot(b.x - a.x, b.y - a.y);
        double control = Math.min(24, d * 0.28);
        double s = Math.toRadians(startDirection), e = Math.toRadians(endDirection);
        return new BezierPath(a,
                new BezierPath.Point(a.x + Math.cos(s) * control, a.y + Math.sin(s) * control),
                new BezierPath.Point(b.x - Math.cos(e) * control, b.y - Math.sin(e) * control), b);
    }

    private static double bearing(BezierPath.Point a, BezierPath.Point b) {
        return Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
    }

    public PlannedRoute plan(Pose2D start, PathSequence requested, double startTravelDirection) {
        clearFailure();
        if (requested == null || requested.isEmpty()) {
            setFailure(FailureReason.EMPTY_SEQUENCE, "No waypoints provided", -1);
            return null;
        }

        // The heading-dependent wall footprint is safe only when the robot
        // keeps the same heading along the ENTIRE route.
        boolean fixedHeading = true;
        for (PathTarget target : requested.targets()) {
            double diff = Math.toDegrees(Math.atan2(
                    Math.sin(Math.toRadians(target.robotHeading - Helper.getPoseHeading(start))),
                    Math.cos(Math.toRadians(target.robotHeading - Helper.getPoseHeading(start)))));
            if (Math.abs(diff) > 1e-6) fixedHeading = false;
        }
        if (wallMargin >= 0 && !fixedHeading) {
            setFailure(FailureReason.HEADING_CHANGE_NOT_SUPPORTED,
                    "Rectangular wall clearance requires a fixed heading", -1);
            return null;
        }

        double wallX = obstacleClearance;
        double wallY = obstacleClearance;
        if (wallMargin >= 0) {
            double a = Math.toRadians(Helper.getPoseHeading(start));
            double halfL = robotLength / 2.0, halfW = robotWidth / 2.0;
            wallX = Math.abs(Math.cos(a)) * halfL + Math.abs(Math.sin(a)) * halfW + wallMargin;
            wallY = Math.abs(Math.sin(a)) * halfL + Math.abs(Math.cos(a)) * halfW + wallMargin;
        }
        AStarPathfinder astar = new AStarPathfinder(fieldSize, grid, wallX, wallY, zones);
        PlannedRoute result = new PlannedRoute();
        BezierPath.Point current = new BezierPath.Point(Helper.getPoseX(start), Helper.getPoseY(start));
        if (!validatePosition(astar, current.x, current.y, true, 0)) return null;
        double heading = Helper.getPoseHeading(start);
        double initialTravel = startTravelDirection;
        int waypoint = 0;
        for (PathTarget target : requested.targets()) {
            waypoint++;
            BezierPath.Point goal = new BezierPath.Point(target.x, target.y);
            if (!validatePosition(astar, goal.x, goal.y, false, waypoint)) return null;
            List<BezierPath.Point> points = astar.find(current, goal);
            if (points == null) {
                setFailure(FailureReason.NO_PATH_FOUND,
                        String.format(Locale.US, "No route from (%.2f, %.2f) to (%.2f, %.2f)",
                                current.x, current.y, goal.x, goal.y), waypoint);
                return null;
            }
            for (int i = 0; i < points.size() - 1; i++) {
                BezierPath.Point a = points.get(i), b = points.get(i + 1);
                if (Math.hypot(b.x - a.x, b.y - a.y) < 1e-6) continue;
                boolean last = i == points.size() - 2;
                double entry = i == 0 ? initialTravel : bearing(points.get(i - 1), b);
                double exit = last ? target.travelDirection : bearing(a, points.get(i + 2));
                BezierPath curve = candidate(a, b, entry, exit);
                if (!safeCurve(curve, astar)) curve = BezierPath.straight(a, b);
                if (!safeCurve(curve, astar)) {
                    setFailure(FailureReason.UNSAFE_CURVE,
                            String.format(Locale.US, "Unsafe segment (%.2f, %.2f) to (%.2f, %.2f)",
                                    a.x, a.y, b.x, b.y), waypoint);
                    return null;
                }
                double endHeading = last ? target.robotHeading : heading;
                WaypointMode mode = last ? target.mode : WaypointMode.PASS_THROUGH;
                result.add(new PathSegment(curve, heading, endHeading, mode));
                heading = endHeading;
            }
            current = goal;
            heading = target.robotHeading;
            initialTravel = target.travelDirection;
        }
        if (result.isEmpty()) {
            setFailure(FailureReason.EMPTY_ROUTE, "Planning produced no path segments", -1);
            return null;
        }
        return result;
    }
}
