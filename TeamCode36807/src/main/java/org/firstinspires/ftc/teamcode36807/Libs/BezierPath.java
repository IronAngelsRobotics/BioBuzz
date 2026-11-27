package org.firstinspires.ftc.teamcode36807.Libs;

public final class BezierPath {
    public static final class Point {
        public final double x,y;
        public Point(double x,double y) { this.x=x; this.y=y; }
    }
    public final Point p0,p1,p2,p3;
    public BezierPath(Point p0,Point p1,Point p2,Point p3) {
        this.p0=p0; this.p1=p1; this.p2=p2; this.p3=p3;
    }
    public static BezierPath straight(Point a,Point b) {
        return new BezierPath(a,lerp(a,b,1.0/3),lerp(a,b,2.0/3),b);
    }
    public static Point lerp(Point a,Point b,double t) {
        return new Point(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t);
    }
    public Point getPoint(double t) {
        t=Math.max(0,Math.min(1,t)); double u=1-t;
        return new Point(u*u*u*p0.x+3*u*u*t*p1.x+3*u*t*t*p2.x+t*t*t*p3.x,
                         u*u*u*p0.y+3*u*u*t*p1.y+3*u*t*t*p2.y+t*t*t*p3.y);
    }
    public double length() {
        double result=0; Point prev=p0;
        for(int i=1;i<=100;i++) { Point p=getPoint(i/100.0);
            result+=Math.hypot(p.x-prev.x,p.y-prev.y); prev=p; }
        return result;
    }
}
