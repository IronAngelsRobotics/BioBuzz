package org.firstinspires.ftc.teamcode36807.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode36807.Libs.Constants;
import org.firstinspires.ftc.teamcode36807.Libs.SmoothServo;
import org.firstinspires.ftc.teamcode36807.Robots.BeezyBot;


/**
 * This opmode is for controlling the BeezyBot.
 * Operation instruction:
 * Gamepad 1:
 *   Left joystick   - move forward/backward and strafe left/right
 *   Right joystick  - turn the robot left/right (rotate)
 *   Left trigger    - toggle shooting: flywheel, middle feeder servo, intake, and funnel up/down
 *   Right trigger   - toggle intake on/off (lowers the funnel first)
 *   Y               - run intake in reverse (clear a jam)
 *   B               - raise/lower funnel (stops intake)
 *   X / A           - increase / decrease flywheel power
 * Gamepad 2:
 *   Left trigger    - same as gamepad 1 left trigger
 *   B               - same as gamepad 1 B
 *
 * The main loop never calls sleep(), so the driver can always steer and the
 * funnel servo always moves smoothly. Actions that need a delay are scheduled
 * as a "pending action" and run later by checking a timer.
 */
@TeleOp(name="Beezy")
public class BeezyBotOpMode extends LinearOpMode {

    // Actions that run after a short delay (instead of using sleep()).
    private enum PendingAction {
        NONE,
        START_FEEDING,          // after flywheel spins up: feeder on, funnel up, intake slow
        START_INTAKE_FORWARD,   // after funnel lowers: intake forward
        START_INTAKE_REVERSE    // after funnel lowers: intake reverse
    }

    private BeezyBot robot;
    private SmoothServo lift;

    // Previous button states, for detecting a new press (edge detection)
    private boolean previousLeftTrigger = false;
    private boolean previousLeftTrigger2 = false;
    private boolean previousRightTrigger = false;
    private boolean previousX = false;
    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousB2 = false;
    private boolean previousY = false;

    private boolean flywheelOn = false;
    private double flywheelPower = Constants.FLYWHEEL_POWER;

    private PendingAction pendingAction = PendingAction.NONE;
    private double pendingDelaySeconds = 0;
    private final ElapsedTime pendingTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new BeezyBot(hardwareMap);
        robot.servoLift.setPosition(Constants.LIFT_START);
        lift = new SmoothServo(robot.servoLift);
        lift.setSpeed(Constants.LIFT_SPEED);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.rgb.green();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double turn    = gamepad1.right_stick_x;

            boolean currentLeftTrigger = gamepad1.left_trigger_pressed;
            boolean currentLeftTrigger2 = gamepad2.left_trigger_pressed;
            boolean currentRightTrigger = gamepad1.right_trigger_pressed;
            boolean currentX = gamepad1.x;
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;
            boolean currentB2 = gamepad2.b;
            boolean currentY = gamepad1.y;

            // Set power to wheels
            robot.drive.setPower(forward, strafe, turn);

            // Each button is checked on its own (not else-if), so a press is never lost
            // because a different button was pressed in the same loop.

            // Left trigger: toggle shooting
            if ((currentLeftTrigger && !previousLeftTrigger) || (currentLeftTrigger2 && !previousLeftTrigger2)) {
                if (!flywheelOn) {
                    startShooting();
                } else {
                    stopShooting();
                }
            }

            // Y: spin intake in reverse
            if (currentY && !previousY) {
                robot.intake.setPower(0);
                lift.setPosition(Constants.LIFT_DOWN);
                schedule(PendingAction.START_INTAKE_REVERSE, Constants.FUNNEL_SETTLE_SECONDS);
            }

            // Right trigger: toggle intake on/off
            if (currentRightTrigger && !previousRightTrigger) {
                boolean intakeOn = robot.intake.getPower() != 0
                        || pendingAction == PendingAction.START_INTAKE_FORWARD
                        || pendingAction == PendingAction.START_INTAKE_REVERSE;
                if (intakeOn) {
                    cancelPending();
                    robot.intake.setPower(0);
                } else {
                    lift.setPosition(Constants.LIFT_DOWN);
                    schedule(PendingAction.START_INTAKE_FORWARD, Constants.FUNNEL_SETTLE_SECONDS);
                }
            }

            // B: raise/lower funnel
            if ((currentB && !previousB) || (currentB2 && !previousB2)) {
                cancelPending();
                robot.intake.setPower(0);
                lift.setPosition(isFunnelUp() ? Constants.LIFT_DOWN : Constants.LIFT_UP);
            }

            // X: increase flywheel power
            if (currentX && !previousX) {
                changeFlywheelPower(Constants.FLYWHEEL_POWER_STEP);
            }

            // A: decrease flywheel power
            if (currentA && !previousA) {
                changeFlywheelPower(-Constants.FLYWHEEL_POWER_STEP);
            }

            runPendingActionIfReady();
            lift.update();

            previousLeftTrigger = currentLeftTrigger;
            previousLeftTrigger2 = currentLeftTrigger2;
            previousRightTrigger = currentRightTrigger;
            previousA = currentA;
            previousB = currentB;
            previousB2 = currentB2;
            previousX = currentX;
            previousY = currentY;

            telemetry.addData("flywheel", flywheelOn ? "ON" : "OFF");
            telemetry.addData("flywheel power", "%.2f", flywheelPower);
            telemetry.addData("pending", pendingAction);
            telemetry.update();
        }

        robot.stop();
    }

    private void startShooting() {
        flywheelOn = true;
        robot.flywheel.setPower(flywheelPower);
        // Let the flywheel spin up before feeding
        schedule(PendingAction.START_FEEDING, Constants.FLYWHEEL_SPINUP_SECONDS);
    }

    private void stopShooting() {
        cancelPending();
        flywheelOn = false;
        robot.servoMiddle.setPower(0);
        robot.flywheel.setPower(0);
        robot.intake.setPower(0);
        lift.setPosition(Constants.LIFT_DOWN);
    }

    // Change the saved flywheel power. Only send it to the motor if the flywheel is running,
    // so pressing X/A while stopped does not start the flywheel.
    private void changeFlywheelPower(double delta) {
        double power = Math.max(0, Math.min(1, flywheelPower + delta));
        // Round to avoid floating point drift (e.g. 0.0000000000003 instead of 0)
        flywheelPower = Math.round(power * 1000) / 1000.0;
        if (flywheelOn) {
            robot.flywheel.setPower(flywheelPower);
        }
    }

    // True if the funnel is up, or moving up. Uses the target, not the in-between position.
    private boolean isFunnelUp() {
        double target = lift.getTargetPosition();
        return Math.abs(target - Constants.LIFT_UP) < Math.abs(target - Constants.LIFT_DOWN);
    }

    private void schedule(PendingAction action, double delaySeconds) {
        pendingAction = action;
        pendingDelaySeconds = delaySeconds;
        pendingTimer.reset();
    }

    private void cancelPending() {
        pendingAction = PendingAction.NONE;
    }

    private void runPendingActionIfReady() {
        if (pendingAction == PendingAction.NONE || pendingTimer.seconds() < pendingDelaySeconds) {
            return;
        }

        switch (pendingAction) {
            case START_FEEDING:
                robot.servoMiddle.setPower(Constants.SERVO_MIDDLE_POWER);
                lift.setPosition(Constants.LIFT_UP);
                robot.intake.setPower(Constants.INTAKE_SHOOT_POWER);
                break;
            case START_INTAKE_FORWARD:
                robot.intake.setPower(Constants.INTAKE_POWER);
                break;
            case START_INTAKE_REVERSE:
                robot.intake.setPower(-Constants.INTAKE_POWER);
                break;
            default:
                break;
        }
        pendingAction = PendingAction.NONE;
    }
}
