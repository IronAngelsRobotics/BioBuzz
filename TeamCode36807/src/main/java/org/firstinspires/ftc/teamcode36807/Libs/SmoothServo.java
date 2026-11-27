package org.firstinspires.ftc.teamcode36807.Libs;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 * Use this class to make servo move smoothly instead of jumping directly to the target.
 */
public class SmoothServo {

    private final Servo servo;
    private final ElapsedTime timer = new ElapsedTime();

    private double currentPosition;
    private double targetPosition;

    // Servo travel speed in position units per second.
    private double speed = 0.5;

    public SmoothServo(Servo servo) {
        this.servo = servo;

        currentPosition = servo.getPosition();
        targetPosition = currentPosition;

        timer.reset();
    }

    // Set the destination without immediately moving the servo.
    public void setPosition(double position) {
        targetPosition = Math.max(0.0, Math.min(1.0, position));
    }

    // Set travel speed (position units per second).
    public void setSpeed(double speed) {
        if (speed <= 0) {
            throw new IllegalArgumentException("Speed must be positive");
        }

        this.speed = speed;
    }

    // Call repeatedly from OpMode.loop().
    public void update() {

        double dt = timer.seconds();
        timer.reset();

        double maxStep = speed * dt;

        double error = targetPosition - currentPosition;

        if (Math.abs(error) <= maxStep) {
            currentPosition = targetPosition;
        } else {
            currentPosition += Math.signum(error) * maxStep;
        }

        servo.setPosition(currentPosition);
    }

    public boolean isMoving() {
        return Math.abs(targetPosition - currentPosition) > 0.0001;
    }

    public double getPosition() {
        return currentPosition;
    }

    // The position the servo is moving toward (last value passed to setPosition).
    public double getTargetPosition() {
        return targetPosition;
    }
}