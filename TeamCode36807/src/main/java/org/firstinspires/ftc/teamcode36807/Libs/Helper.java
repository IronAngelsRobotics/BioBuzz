package org.firstinspires.ftc.teamcode36807.Libs;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public final class Helper {

    /**
     * *
     * @param x  X coordinate in inches
     * @param y  Y coordinate in inches
     * @param heading  Heading in degrees
     * @return Pose2d Object
     */
    public static Pose2D getPose2D(double x, double y, double heading){
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }

    public static double getPoseX(Pose2D pose){
        return pose.getX(DistanceUnit.INCH);
    }

    public static double getPoseY(Pose2D pose){
        return pose.getY(DistanceUnit.INCH);
    }

    public static double getPoseHeading(Pose2D pose){
        return pose.getHeading(AngleUnit.DEGREES);
    }

    private Helper(){
        // final prevents other classes from extending Positions.
        // private Positions() prevents other classes from creating instances.
        // static allows members to be accessed without creating an object.
    }
}
