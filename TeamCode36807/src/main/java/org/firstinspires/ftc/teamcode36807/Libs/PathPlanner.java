package org.firstinspires.ftc.teamcode36807.Libs;

import java.util.ArrayList;
import java.util.List;

/** Static obstacle planner. Fails closed: returns null when no verified route exists. */
public final class PathPlanner {
    private final List<RestrictedZone> zones=new ArrayList<>();
    private final double fieldSize,clearance,grid;
    public PathPlanner(double fieldSize,double robotWidth,double robotLength,double extraMargin,double grid) {
        this.fieldSize=fieldSize; this.grid=grid;
        this.clearance=Math.hypot(robotWidth/2,robotLength/2)+extraMargin;
    }
    public void addRestrictedZone(RestrictedZone zone) { zones.add(zone.expanded(clearance)); }
    public void clearRestrictedZones() { zones.clear(); }
    private boolean safeCurve(BezierPath curve,AStarPathfinder astar) {
        BezierPath.Point last=curve.p0;
        // Segment-vs-rectangle collision tests between samples, not just sample points.
        for(int i=1;i<=150;i++) {
            BezierPath.Point next=curve.getPoint(i/150.0);
            if(!astar.clearLine(last,next)) return false;
            last=next;
        }
        return true;
    }
    private BezierPath candidate(BezierPath.Point a,BezierPath.Point b,
                                 double startDirection,double endDirection) {
        double d=Math.hypot(b.x-a.x,b.y-a.y);
        double control=Math.min(24,d*0.28);
        double s=Math.toRadians(startDirection),e=Math.toRadians(endDirection);
        return new BezierPath(a,
            new BezierPath.Point(a.x+Math.cos(s)*control,a.y+Math.sin(s)*control),
            new BezierPath.Point(b.x-Math.cos(e)*control,b.y-Math.sin(e)*control),b);
    }
    private static double bearing(BezierPath.Point a,BezierPath.Point b) {
        return Math.toDegrees(Math.atan2(b.y-a.y,b.x-a.x));
    }
    public PlannedRoute plan(Pose2D start,PathSequence requested,double startTravelDirection) {
        if(requested==null || requested.isEmpty()) return null;
        AStarPathfinder astar=new AStarPathfinder(fieldSize,grid,clearance,zones);
        PlannedRoute result=new PlannedRoute();
        BezierPath.Point current=new BezierPath.Point(start.x,start.y);
        double heading=start.heading;
        double initialTravel=startTravelDirection;
        for(PathTarget target:requested.targets()) {
            BezierPath.Point goal=new BezierPath.Point(target.x,target.y);
            // Handle rotation in place when X/Y coordinates are unchanged.
            if (Math.hypot(goal.x - current.x, goal.y - current.y) < 1e-6) {

                if (Math.abs((target.robotHeading - heading + 540) % 360 - 180) > 1e-6) {
                    result.add(new PathSegment(
                            BezierPath.straight(current, goal),
                            heading,
                            target.robotHeading,
                            WaypointMode.STOP
                    ));
                }

                heading = target.robotHeading;

                // After rotating in place, the next path should begin
                // in the direction the robot is now facing.
                initialTravel = target.robotHeading;

                continue;
            }

            List<BezierPath.Point> points=astar.find(current,goal);
            if(points==null) return null;
            for(int i=0;i<points.size()-1;i++) {
                BezierPath.Point a=points.get(i),b=points.get(i+1);
                double d=Math.hypot(b.x-a.x,b.y-a.y);
                if (d < 1e-6) {
                    if (Math.abs(target.robotHeading - heading) > 1e-6) {
                        result.add(new PathSegment(
                                BezierPath.straight(a, b),
                                heading,
                                target.robotHeading,
                                WaypointMode.STOP
                        ));
                    }

                    heading = target.robotHeading;
                    continue;
                }

                boolean last=(i==points.size()-2);
                double entry=i==0?initialTravel:bearing(points.get(i-1),b);
                double exit=last?target.travelDirection:bearing(a,points.get(i+2));

                // Use a straight path when the requested travel directions
                // match the direction from the start to the destination.
                double directBearing = bearing(a, b);

                boolean straightMovement =
                        Math.abs(angleDifference(entry, directBearing)) < 1.0
                                && Math.abs(angleDifference(exit, directBearing)) < 1.0;

//                BezierPath curve = straightMovement
//                        ? BezierPath.straight(a, b)
//                        : candidate(a, b, entry, exit);

                // Diagnostic: force all movement segments to be straight.
                BezierPath curve = BezierPath.straight(a, b);

                if (!safeCurve(curve, astar)) {
                    curve = BezierPath.straight(a, b);
                }

                if(!safeCurve(curve,astar)) return null;
                // Intermediate segments inherit current heading, with no forced stop.
                double endHeading=last?target.robotHeading:heading;
                WaypointMode mode=last?target.mode:WaypointMode.PASS_THROUGH;

                // Keep directly aligned movement straight.
                double dx = b.x - a.x;
                double dy = b.y - a.y;

                if (Math.abs(dy) < 0.01 || Math.abs(dx) < 0.01) {
                    curve = BezierPath.straight(a, b);
                }

                result.add(new PathSegment(curve,heading,endHeading,mode));
                heading=endHeading;
            }
            current=goal;
            heading=target.robotHeading;
            initialTravel=target.travelDirection;
        }
        return result.isEmpty()?null:result;
    }

    private static double angleDifference(double a, double b) {
        double difference = (a - b) % 360.0;

        if (difference > 180) difference -= 360;
        if (difference < -180) difference += 360;

        return difference;
    }
}
