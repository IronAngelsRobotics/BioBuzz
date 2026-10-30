package org.firstinspires.ftc.teamcode36807.Libs;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

/** 8-neighbor A* on a 3-inch grid. All cells/edges are checked for clearance. */
public final class AStarPathfinder {
    private final double fieldSize,grid;
    private final List<RestrictedZone> zones;
    private final double minCenter,maxCenter;
    private final int count;
    public AStarPathfinder(double fieldSize,double grid,double clearance,List<RestrictedZone> zones) {
        this.fieldSize=fieldSize; this.grid=grid; this.zones=zones;
        minCenter=clearance; maxCenter=fieldSize-clearance;
        count=(int)Math.floor((maxCenter-minCenter)/grid)+1;
        if(count<2) throw new IllegalArgumentException("Grid too large for field");
    }
    private int index(int x,int y) { return y*count+x; }
    private BezierPath.Point point(int x,int y) {
        return new BezierPath.Point(minCenter+x*grid,minCenter+y*grid);
    }
    private boolean free(double x,double y) {
        if(x<minCenter || x>maxCenter || y<minCenter || y>maxCenter) return false;
        for(RestrictedZone z:zones) if(z.contains(x,y)) return false;
        return true;
    }
    public boolean clearLine(BezierPath.Point a,BezierPath.Point b) {
        if(!free(a.x,a.y)||!free(b.x,b.y)) return false;
        for(RestrictedZone z:zones) if(z.intersects(a.x,a.y,b.x,b.y)) return false;
        return true;
    }
    private int closestVisibleNode(BezierPath.Point p) {
        int best=-1; double bestD=Double.POSITIVE_INFINITY;
        for(int y=0;y<count;y++) for(int x=0;x<count;x++) {
            BezierPath.Point q=point(x,y);
            if(!clearLine(p,q)) continue;
            double d=Math.hypot(q.x-p.x,q.y-p.y);
            if(d<bestD) { bestD=d; best=index(x,y); }
        }
        return best;
    }
    private static final class Node {
        final int id; final double f;
        Node(int id,double f) { this.id=id; this.f=f; }
    }
    public List<BezierPath.Point> find(BezierPath.Point start,BezierPath.Point goal) {
        if(!free(start.x,start.y)||!free(goal.x,goal.y)) return null;
        if(clearLine(start,goal)) return Arrays.asList(start,goal);
        int s=closestVisibleNode(start),g=closestVisibleNode(goal);
        if(s<0||g<0) return null;
        int n=count*count;
        double[] dist=new double[n]; Arrays.fill(dist,Double.POSITIVE_INFINITY);
        int[] prev=new int[n]; Arrays.fill(prev,-1);
        boolean[] closed=new boolean[n];
        PriorityQueue<Node> open=new PriorityQueue<>(Comparator.comparingDouble(a->a.f));
        dist[s]=0; open.add(new Node(s,0));
        int[] moves={-1,0,1};
        while(!open.isEmpty()) {
            Node node=open.poll(); int id=node.id;
            if(closed[id]) continue;
            closed[id]=true;
            if(id==g) break;
            int x=id%count,y=id/count;
            BezierPath.Point a=point(x,y);
            for(int dy:moves) for(int dx:moves) {
                if(dx==0&&dy==0) continue;
                int xx=x+dx,yy=y+dy;
                if(xx<0||yy<0||xx>=count||yy>=count) continue;
                int ni=index(xx,yy); if(closed[ni]) continue;
                BezierPath.Point b=point(xx,yy);
                if(!clearLine(a,b)) continue;
                double alt=dist[id]+Math.hypot(b.x-a.x,b.y-a.y);
                if(alt<dist[ni]) {
                    dist[ni]=alt; prev[ni]=id;
                    BezierPath.Point goalGrid=point(g%count,g/count);
                    open.add(new Node(ni,alt+Math.hypot(b.x-goalGrid.x,b.y-goalGrid.y)));
                }
            }
        }
        if(!closed[g]) return null;
        List<BezierPath.Point> path=new ArrayList<>();
        for(int id=g;id>=0;id=prev[id]) { path.add(point(id%count,id/count)); if(id==s) break; }
        Collections.reverse(path);
        path.add(0,start); path.add(goal);
        // Line-of-sight simplification, still collision-checked.
        List<BezierPath.Point> simple=new ArrayList<>();
        int i=0; simple.add(path.get(0));
        while(i<path.size()-1) {
            int j=path.size()-1;
            while(j>i+1 && !clearLine(path.get(i),path.get(j))) j--;
            simple.add(path.get(j)); i=j;
        }
        return simple;
    }
}
