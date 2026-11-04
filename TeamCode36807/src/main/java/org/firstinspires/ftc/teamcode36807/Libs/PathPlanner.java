package org.firstinspires.ftc.teamcode36807.Libs;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

/** Static obstacle planner. Returns null when no collision-free route exists. */
public final class PathPlanner {
    private final List<RestrictedZone> zones = new ArrayList<>();
    private final double fieldSize, obstacleClearance, grid;
    private final double robotWidth, robotLength, wallMargin;

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
        if (requested == null || requested.isEmpty()) return null;

        // The heading-dependent wall footprint is safe only when the robot
        // keeps the same heading along the ENTIRE route.
        boolean fixedHeading = true;
        for (PathTarget target : requested.targets()) {
            double diff = Math.toDegrees(Math.atan2(
                    Math.sin(Math.toRadians(target.robotHeading - Helper.getPoseHeading(start))),
                    Math.cos(Math.toRadians(target.robotHeading - Helper.getPoseHeading(start)))));
            if (Math.abs(diff) > 1e-6) fixedHeading = false;
        }
        if (wallMargin >= 0 && !fixedHeading) return null;

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
        double heading = Helper.getPoseHeading(start);
        double initialTravel = startTravelDirection;
        for (PathTarget target : requested.targets()) {
            BezierPath.Point goal = new BezierPath.Point(target.x, target.y);
            List<BezierPath.Point> points = astar.find(current, goal);
            if (points == null) return null;
            for (int i = 0; i < points.size() - 1; i++) {
                BezierPath.Point a = points.get(i), b = points.get(i + 1);
                if (Math.hypot(b.x - a.x, b.y - a.y) < 1e-6) continue;
                boolean last = i == points.size() - 2;
                double entry = i == 0 ? initialTravel : bearing(points.get(i - 1), b);
                double exit = last ? target.travelDirection : bearing(a, points.get(i + 2));
                BezierPath curve = candidate(a, b, entry, exit);
                if (!safeCurve(curve, astar)) curve = BezierPath.straight(a, b);
                if (!safeCurve(curve, astar)) return null;
                double endHeading = last ? target.robotHeading : heading;
                WaypointMode mode = last ? target.mode : WaypointMode.PASS_THROUGH;
                result.add(new PathSegment(curve, heading, endHeading, mode));
                heading = endHeading;
            }
            current = goal;
            heading = target.robotHeading;
            initialTravel = target.travelDirection;
        }
        return result.isEmpty() ? null : result;
    }
}
