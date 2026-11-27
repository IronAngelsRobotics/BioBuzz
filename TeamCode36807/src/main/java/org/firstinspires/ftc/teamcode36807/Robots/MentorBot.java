package org.firstinspires.ftc.teamcode36807.Robots;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode36807.Libs.MecanumDrive;
import org.firstinspires.ftc.teamcode36807.Libs.RobotSize;

import java.net.DatagramPacket;

/*
This class represents the mentor's robot.
 */
public class MentorBot {
    public final MecanumDrive drive;
    public final GoBildaPinpointDriver odom;
    public final DcMotorEx intake;
    public final RobotSize size = new RobotSize(17.5, 16.125); // include size of left and right wheels


    public MentorBot(HardwareMap hardwareMap){
        drive = new MecanumDrive(hardwareMap);
        odom = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Motor (wheel) should rotate in the forward direction, relative to the robot,
        // when positive power is applied.
        drive.setDirection(MecanumDrive.Wheel.BACK_RIGHT, DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake when power is zero
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
        odom.setOffsets(-92.25, -184.7875, DistanceUnit.MM);

        // Don't reset here. Call resetPosAndIMU() in the OpMode
        // odom.resetPosAndIMU();
    }

    public void setPower(double forward, double strafe, double turn){
        drive.setPower(forward, strafe, turn);
    }

    public void stop(){
        drive.setPower(0, 0, 0);
        intake.setPower(0);
    }

}
