package org.firstinspires.ftc.teamcode36807.Libs;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class PathSequence {
    private final List<PathTarget> targets=new ArrayList<>();
    public PathSequence add(double x,double y,double travelDirection,double robotHeading,WaypointMode mode) {
        targets.add(new PathTarget(x,y,travelDirection,robotHeading,mode));
        return this;
    }
    public List<PathTarget> targets() { return Collections.unmodifiableList(targets); }
    public boolean isEmpty() { return targets.isEmpty(); }
}
