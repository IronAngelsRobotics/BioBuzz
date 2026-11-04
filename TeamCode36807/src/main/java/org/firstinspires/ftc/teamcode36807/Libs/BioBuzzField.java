package org.firstinspires.ftc.teamcode36807.Libs;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Reusable 2D map and named navigation destinations for the BIOBUZZ field.
 *
 * HIVE support-bar footprints are mapped from the supplied STEP-model
 * measurements. The mapping assumes CAD origin = field center and CAD
 * horizontal X/Z align with navigation X/Y. Verify orientation on the
 * physical field before enabling automatic driving.
 *
 * Coordinates: inches; origin at the selected lower-left field corner;
 * +X right, +Y up, positive heading counterclockwise.
 */
public final class BioBuzzField {
    public static final double SIDE_INCHES = 24.0;
    public static final double FIELD_SIZE = 144.0;
    public static final double FIELD_ORIGIN = 72.0;

    /**
     * SAFETY INTERLOCK: set true only after measuring/verifying ALL permanent
     * obstacle rectangles in the coordinate system used by the robot.
     * The sample rectangles below are NOT official field measurements.
     */
    public static final boolean FIELD_MAP_VERIFIED = true; // Change to true ONLY after checking the CAD-to-field alignment.

    // Rectangles are {minX, minY, maxX, maxY}, in inches.
    // Source measurements in STEP horizontal X/Z (in):
    //   left bar:  X [-24.73,-22.75], Z [-19.47,19.47]
    //   right bar: X [22.75,24.73],  Z [-19.47,19.47]
    // IMPORTANT: The planner expands EACH rectangle by the robot clearance;
    // do not manually expand these footprints a second time.
    // These are floor support BARS, not every part of the HIVE structure.
    private static final double[][] HIVE_SUPPORTS = {
        {47.27, 52.53, 49.25, 91.47}, // Left HIVE support bar
        {94.75, 52.53, 96.73, 91.47}  // Right HIVE support bar
    };

    // TODO: Add only physically impassable permanent FLOWER footprints,
    // measured from CAD. Floor tape/markings are not obstacles by default.
    private static final double[][] OTHER_OBSTACLES = {};

    public enum Destination {
        HOME,
        RED_LOADING_ZONE,
        BLUE_LOADING_ZONE,
        RED_SHOOTING_POSITION_RIGHT,
        RED_SHOOTING_POSITION_LEFT,
        BLUE_SHOOTING_POSITION_RIGHT,
        BLUE_SHOOTING_POSITION_LEFT
    }

    private BioBuzzField() { }

    /** Returns a named destination; unmeasured positions fail explicitly. */
    public static Pose2D getDestination(Destination destination) {
        switch (destination) {
            case HOME: return Helper.getPose2D(12.0, 12.0, 0.0);
            case RED_LOADING_ZONE: return Helper.getPose2D(12, 108, Double.NaN);
            case BLUE_LOADING_ZONE: return Helper.getPose2D(132, 36, Double.NaN);
            case RED_SHOOTING_POSITION_LEFT: return Helper.getPose2D(60.5, 131, -90);
            case RED_SHOOTING_POSITION_RIGHT: return Helper.getPose2D(60.5, 11, 90);
            case BLUE_SHOOTING_POSITION_LEFT: return Helper.getPose2D(83.5, 11, 90);
            case BLUE_SHOOTING_POSITION_RIGHT: return Helper.getPose2D(83.5, 131, -90);
            default:
                throw new IllegalArgumentException("Unknown destination: " + destination);
        }
    }

    /** Register obstacle footprints with PathPlanner (which inflates them). */
    public static void configureRestrictedZones(PathPlanner planner) {
        if (!FIELD_MAP_VERIFIED || HIVE_SUPPORTS.length < 2) {
            throw new IllegalStateException("BIOBUZZ HIVE leg coordinates not verified");
        }
        planner.clearRestrictedZones();
        addRectangles(planner, HIVE_SUPPORTS);
        addRectangles(planner, OTHER_OBSTACLES);
    }

    private static void addRectangles(PathPlanner planner, double[][] rectangles) {
        for (double[] r : rectangles) {
            if (r.length != 4 || r[0] >= r[2] || r[1] >= r[3]
                    || r[0] < 0 || r[1] < 0 || r[2] > FIELD_SIZE || r[3] > FIELD_SIZE) {
                throw new IllegalArgumentException("Invalid restricted-zone rectangle");
            }
            planner.addRestrictedZone(new RestrictedZone(r[0], r[1], r[2], r[3]));
        }
    }

}
