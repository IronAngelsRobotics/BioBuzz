package org.firstinspires.ftc.teamcode36807.Libs;

public final class RestrictedZone {
    public final double minX,minY,maxX,maxY;
    public RestrictedZone(double minX,double minY,double maxX,double maxY) {
        if(minX>=maxX || minY>=maxY) throw new IllegalArgumentException("Invalid rectangle");
        this.minX=minX; this.minY=minY; this.maxX=maxX; this.maxY=maxY;
    }
    public RestrictedZone expanded(double d) {
        return new RestrictedZone(minX-d,minY-d,maxX+d,maxY+d);
    }
    public boolean contains(double x,double y) {
        return x>=minX && x<=maxX && y>=minY && y<=maxY;
    }
    // Liang-Barsky line-vs-rectangle intersection; touching boundary is blocked.
    public boolean intersects(double x0,double y0,double x1,double y1) {
        double dx=x1-x0,dy=y1-y0;
        double[] p={-dx,dx,-dy,dy};
        double[] q={x0-minX,maxX-x0,y0-minY,maxY-y0};
        double lo=0,hi=1;
        for(int i=0;i<4;i++) {
            if(Math.abs(p[i])<1e-12) { if(q[i]<0) return false; }
            else {
                double t=q[i]/p[i];
                if(p[i]<0) lo=Math.max(lo,t); else hi=Math.min(hi,t);
                if(lo>hi) return false;
            }
        }
        return true;
    }
}
