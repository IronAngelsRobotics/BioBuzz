package org.firstinspires.ftc.teamcode36807.Robots;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode36807.Libs.MecanumDrive;
import org.firstinspires.ftc.teamcode36807.Libs.RGBIndicator;
import org.firstinspires.ftc.teamcode36807.Libs.RobotSize;

/*
This class represents the students' Breezy robot.
 */
public class BeezyBot {
    public final MecanumDrive drive;
    public final GoBildaPinpointDriver odom;
    public final DcMotor flywheel;
    public final DcMotor intake;
    public final RGBIndicator rgb;
    public final Servo servoLift;
    public final RobotSize size = new RobotSize(17, 15.25); //width include size of left and right wheels


    public BeezyBot(HardwareMap hardwareMap){
        drive = new MecanumDrive(hardwareMap);
        odom = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        rgb = new RGBIndicator(hardwareMap, "rgb");
        servoLift = hardwareMap.get(Servo.class, "servoLift");
;
        // Motor (wheel) should rotate in the forward direction, relative to the robot,
        // when positive power is applied.
        drive.setDirection(MecanumDrive.Wheel.FRONT_LEFT, DcMotorSimple.Direction.REVERSE);
        drive.setDirection(MecanumDrive.Wheel.BACK_LEFT, DcMotorSimple.Direction.REVERSE);

        // Brake when power is zero
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* Configure Offset:
         * Sets the odometry pod positions relative to the point that the odometry computer tracks around.
         * The most common tracking position is the center of the robot.
         * The X pod offset refers to how far sideways (in mm) from the tracking point the X (forward)
         * odometry pod is. Left of the center is a positive number, right of center is a negative number.
         * the Y pod offset refers to how far forwards (in mm) from the tracking point the Y (strafe) odometry
         * pod is. forward of center is a positive number, backwards is a negative number.
         * xOffset - how sideways from the center of the robot is the X (forward) pod? Left increases
         * yOffset - how far forward from the center of the robot is the Y (Strafe) pod? forward increases
         */
        odom.setOffsets(-76.2, -167, DistanceUnit.MM);

        /* Configure encoder direction:
         * X position should increase when robot moves forward.
         * Y position should increase when robot moves left.
         */
        odom.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Don't reset here. Call resetPosAndIMU() in the OpMode
        // odom.resetPosAndIMU();
    }

    private void init(){
        stop();
        flywheel.setPower(0);
        intake.setPower(0);
        servoLift.setPosition(0.5); // 0 degree
        rgb.off();
    }

    public void setPower(double forward, double strafe, double turn){
        drive.setPower(forward, strafe, turn);
    }

    /*
    * Stop motors and intake
     */
    public void stop(){
        drive.setPower(0, 0, 0);
        intake.setPower(0);
    }

    /*
    * Stop all motors and reset robot to initial configuration.
     */
    public void off()
    {
        stop();
        servoLift.setPosition(0.5);
        rgb.off();
    }

}
