package org.firstinspires.ftc.teamcode36807.Libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * A minimal FTC mecanum drivetrain class. No pathing, no odometry, and no PID.
 */
public class MecanumDrive {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    public MecanumDrive(HardwareMap hardwareMap){
        this(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight");

    }

    public MecanumDrive(HardwareMap hardwareMap, String wheel_frontLeft, String wheel_frontRight, String wheel_backLeft, String wheel_backRight) {

        frontLeft  = hardwareMap.get(DcMotorEx.class, wheel_frontLeft);
        frontRight = hardwareMap.get(DcMotorEx.class, wheel_frontRight);
        backLeft   = hardwareMap.get(DcMotorEx.class, wheel_backLeft);
        backRight  = hardwareMap.get(DcMotorEx.class, wheel_backRight);

        // Typical mecanum motor directions.
        // Reverse these if your physical motor orientation is different.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Brake when power is zero
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * @param forward  + = forward, - = backward
     * @param strafe   + = right,   - = left
     * @param turn     + = clockwise, - = counter-clockwise
     */
    public void drive(double forward, double strafe, double turn) {

        double frontLeftPower  = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower   = forward - strafe + turn;
        double backRightPower  = forward + strafe - turn;


        // Normalize powers so none exceed 1.0
        double max = Math.max(
                Math.abs(frontLeftPower),
                Math.max(
                        Math.abs(frontRightPower),
                        Math.max(
                                Math.abs(backLeftPower),
                                Math.abs(backRightPower)
                        )
                )
        );

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void set_motor_direction(String name, DcMotor.Direction direction){
        // ToDO:
    }

    public void stop() {
        drive(0, 0, 0);
    }
}