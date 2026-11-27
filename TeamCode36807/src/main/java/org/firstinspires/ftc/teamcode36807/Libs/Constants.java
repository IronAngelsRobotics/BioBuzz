package org.firstinspires.ftc.teamcode36807.Libs;

public final class Constants {

    // Robot initial configuration
    public static final double FLYWHEEL_POWER = 0.61;
    public static final double SERVO_MIDDLE_POWER = -0.9;
    public static final double LIFT_UP = 0.47; // decrease to raise the lift/funnel up
    public static final double LIFT_DOWN = 0.522; // increase to lower lift/funnel down
    public static final double LIFT_START = 0.5; // lift/funnel position during init

    // TeleOp tuning
    public static final double INTAKE_POWER = 0.40;          // intake on (and reverse with Y)
    public static final double INTAKE_SHOOT_POWER = 0.2;     // intake while shooting
    public static final double FLYWHEEL_POWER_STEP = 0.01;   // X / A adjust step
    public static final double LIFT_SPEED = 0.3;             // SmoothServo speed (position units / sec)
    public static final double FLYWHEEL_SPINUP_SECONDS = 0.5; // wait before feeding when shooting starts
    public static final double FUNNEL_SETTLE_SECONDS = 0.25;  // wait for funnel to lower before intake runs


    // Prevent creating an instance of this class
    private Constants() {}
}
