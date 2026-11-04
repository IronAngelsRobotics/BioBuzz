package org.firstinspires.ftc.teamcode36807.Libs;


import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Predefined Alliance field positions.
 */
public final class Positions {

    /**
     * Available alliances.
     */
    public enum Alliance {
        RED,
        BLUE
    }

    /**
     * Named destinations for an alliance.
     */
    public static class AlliancePositions {

        public final Pose2D park;
        public final Pose2D shooting1;
        public final Pose2D shooting2;

        private AlliancePositions(Pose2D park, Pose2D shooting1, Pose2D shooting2) {
            this.park = park;
            this.shooting1 = shooting1;
            this.shooting2 = shooting2;
        }
    }

    /**
     * BLUE alliance destinations.
     */
    public static final AlliancePositions BLUE =
            new AlliancePositions(
                    Helper.getPose2D(132, 36, Double.NaN),   // Park
                    Helper.getPose2D(84, 7.625, 90),         // Shooting 1
                    Helper.getPose2D(84, 136.375, -90)       // Shooting 2
            );

    /**
     * RED alliance destinations.
     *
     */
    public static final AlliancePositions RED =
            new AlliancePositions(
                    Helper.getPose2D(12, 108, Double.NaN), // Park
                    Helper.getPose2D(60, 136.375, -90),    // Shooting 1
                    Helper.getPose2D(60, 7.625, 90)        // Shooting 2
            );

    /**
     * Returns the destinations for the selected alliance.
     */
    public static AlliancePositions get(Alliance alliance) {

        if (alliance == null) {
            throw new IllegalArgumentException("Alliance cannot be null");
        }

        switch (alliance) {
            case RED:
                return RED;
            case BLUE:
                return BLUE;
            default:
                throw new IllegalArgumentException(
                        "Unsupported alliance: " + alliance
                );
        }
    }

    private Positions() {
        // final prevents other classes from extending Positions.
        // private Positions() prevents other classes from creating instances.
        // static allows members to be accessed without creating an object.
    }
}