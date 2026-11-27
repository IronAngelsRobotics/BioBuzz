package org.firstinspires.ftc.teamcode36807.Libs;

public final class PathTarget {
    public final double x,y,travelDirection,robotHeading;
    public final WaypointMode mode;
    public PathTarget(double x,double y,double travelDirection,double robotHeading,WaypointMode mode) {
        this.x=x; this.y=y; this.travelDirection=travelDirection;
        this.robotHeading=robotHeading; this.mode=mode;
    }
}
