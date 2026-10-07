package org.firstinspires.ftc.teamcode36807.OpModes.Examples;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * Simple OpModes to check a Gobilda odometry.
 * Note: manually push the robot forward/backward or left/right
 */


//@Autonomous(name="MyAutonomous")
@TeleOp(name="Odom")
//@Disabled
public class OdometryDemo extends LinearOpMode {

    @Override
    public void runOpMode() {

        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // IMPORTANT:
        // Put your actual pod offsets here.
        // X offset = forward/backward pod position
        // Y offset = sideways pod position
        pinpoint.setOffsets(
                0,
                0,
                DistanceUnit.MM
        );

        // Change these to match your odometry pods.
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // You may need to reverse one or both depending
        // on how your pods are physically mounted.
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();

        telemetry.addLine("Pinpoint ready");
        telemetry.addLine("Place robot at starting position.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Must be called before reading position.
            pinpoint.update();

            double x = pinpoint.getPosX(DistanceUnit.INCH);
            double y = pinpoint.getPosY(DistanceUnit.INCH);

            double heading =
                    pinpoint.getHeading(AngleUnit.DEGREES);

            double velocityX =
                    pinpoint.getVelX(DistanceUnit.INCH);

            double velocityY =
                    pinpoint.getVelY(DistanceUnit.INCH);

            telemetry.addLine("=== PINPOINT TEST ===");

            telemetry.addData("X", "%.2f in", x);
            telemetry.addData("Y", "%.2f in", y);
            telemetry.addData("Heading", "%.1f deg", heading);

            telemetry.addLine();
            telemetry.addData("X Velocity", "%.2f in/s", velocityX);
            telemetry.addData("Y Velocity", "%.2f in/s", velocityY);

            telemetry.update();
        }
    }
}
