package org.firstinspires.ftc.teamcode36807.Libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * A minimal FTC mecanum drivetrain class. No pathing, no odometry, and no PID.
 */
public class MecanumDrive {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    public enum Wheel{
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    /**
     * Instantiate class with default names
     *
     * <p>- Call method setDirection() to reverse direction of a wheel's rotation as needed.</p>
     **/
    public MecanumDrive(HardwareMap hardwareMap){
        this(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight");

    }

    /**
     * Instantiate class with provided wheel names
     *
     * <p>- Call method setDirection() to reverse direction of a wheel's rotation as needed.</p>
     *
     */
    public MecanumDrive(HardwareMap hardwareMap, String wheel_frontLeft, String wheel_frontRight, String wheel_backLeft, String wheel_backRight) {

        frontLeft  = hardwareMap.get(DcMotorEx.class, wheel_frontLeft);
        frontRight = hardwareMap.get(DcMotorEx.class, wheel_frontRight);
        backLeft   = hardwareMap.get(DcMotorEx.class, wheel_backLeft);
        backRight  = hardwareMap.get(DcMotorEx.class, wheel_backRight);

        // Typical mecanum motor directions. Motor should rotate in the forward direction, relative to the robot,
        // when positive power is applied.
        // Don't change direction here. Instead, caller should call setDirection().
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Brake when power is zero
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }

    /**
     * @param forward  + = forward, - = backward
     * @param strafe   + = right,   - = left
     * @param turn     + = clockwise, - = counter-clockwise
     */
    public void setPower(double forward, double strafe, double turn) {

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

    public DcMotorEx getWheel(Wheel wheel){
        DcMotorEx motor = null;

        switch(wheel){
            case FRONT_LEFT:
                motor = frontLeft;
                break;
            case FRONT_RIGHT:
                motor = frontRight;
                break;
            case BACK_LEFT:
                motor = backLeft;
                break;
            case BACK_RIGHT:
                motor = backRight;
                break;
            default:
                throw new IllegalArgumentException("Invalid wheel: " + wheel);
        }

        if (motor == null) {
            throw new IllegalStateException("Motor not initialized for wheel: " + wheel);
        }

        return motor;
    }

    /**
     * Reverse motor rotation so you don't need to change the physical orientation of the motor mount.
     **/
    public void setDirection(Wheel wheel, DcMotor.Direction direction){
        switch(wheel){
            case FRONT_LEFT:
                frontLeft.setDirection(direction);
                break;
            case FRONT_RIGHT:
                frontRight.setDirection(direction);
                break;
            case BACK_LEFT:
                backLeft.setDirection(direction);
                break;
            case BACK_RIGHT:
                backRight.setDirection(direction);
                break;
            default:
                throw new IllegalArgumentException("Invalid wheel: " + wheel);
        }
    }

    public void stop() {
        setPower(0, 0, 0);
    }
}