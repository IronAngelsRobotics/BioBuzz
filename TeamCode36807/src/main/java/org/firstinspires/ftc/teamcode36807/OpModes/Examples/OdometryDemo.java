package org.firstinspires.ftc.teamcode36807.OpModes.Examples;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode36807.Robots.BeezyBot;


/**
 * Simple OpModes to check a GoBilda odometry.
 * Do not drive the robot.
 * Push the robot forward: X values should increase
 * Push the robot left: y value should increase.
 * Manually rotate the robot inplace: X & Y should remain essentially unchanged
 * while the heading changes.
 *
 * Reconfigure the odometry direction and offset as necessary.
 *
 *
 */

@TeleOp(name="Odometry")
@Disabled
public class OdometryDemo extends LinearOpMode {
    BeezyBot robot;

    @Override
    public void runOpMode() {
        // Call robot.odom.setEncoderDirections() to reverse to x/y as necessary.
        robot = new BeezyBot(hardwareMap);
        robot.odom.resetPosAndIMU();

        telemetry.addLine("Pinpoint ready");
        telemetry.addLine("Place robot at starting position.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Must be called before reading position.
            robot.odom.update();

            double x = robot.odom.getPosX(DistanceUnit.INCH);
            double y = robot.odom.getPosY(DistanceUnit.INCH);

            double heading = robot.odom.getHeading(AngleUnit.DEGREES);
            double velocityX = robot.odom.getVelX(DistanceUnit.INCH);
            double velocityY = robot.odom.getVelY(DistanceUnit.INCH);

            telemetry.addLine("=== PINPOINT TEST ===");
            telemetry.addData("RobotX", "%.2f in", x);
            telemetry.addData("RobotY", "%.2f in", y);
            telemetry.addData("Heading", "%.1f deg", heading);

            telemetry.addLine();
            telemetry.addData("X Velocity", "%.2f in/s", velocityX);
            telemetry.addData("Y Velocity", "%.2f in/s", velocityY);

            telemetry.update();
        }
    }
}
