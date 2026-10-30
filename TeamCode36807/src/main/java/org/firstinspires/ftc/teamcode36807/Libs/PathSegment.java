package org.firstinspires.ftc.teamcode36807.Libs;

public final class PathSegment {
    public final BezierPath curve;
    public final double startHeading,endHeading;
    public final WaypointMode mode;
    public PathSegment(BezierPath curve,double startHeading,double endHeading,WaypointMode mode) {
        this.curve=curve; this.startHeading=startHeading;
        this.endHeading=endHeading; this.mode=mode;
    }

    public boolean isRotationOnly() {
        return Math.hypot(
                curve.p3.x - curve.p0.x,
                curve.p3.y - curve.p0.y
        ) < 1e-6;
    }
}
