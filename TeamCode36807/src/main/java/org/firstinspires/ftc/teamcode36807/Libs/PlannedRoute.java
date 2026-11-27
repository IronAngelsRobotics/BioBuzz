package org.firstinspires.ftc.teamcode36807.Libs;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class PlannedRoute {
    private final List<PathSegment> segments=new ArrayList<>();
    public void add(PathSegment s) { segments.add(s); }
    public List<PathSegment> segments() { return Collections.unmodifiableList(segments); }
    public boolean isEmpty() { return segments.isEmpty(); }
}
