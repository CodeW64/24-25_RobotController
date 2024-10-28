package org.firstinspires.ftc.teamcode.teamprograms;

import java.util.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamprograms.auto.AutoInit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="Mecanum Positioning OpMode (TestOp)", group="teamcode")
public class TestOp extends AutoInit {
    @Override 
    public void opMode_start() {
        // Nothing imporetanat
    } 

    @Override 
    public void opMode_loop() {
        driveWheels();

        final YawPitchRollAngles data = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Imu Roll  (x+)", data.getRoll());
        telemetry.addData("Imu Pitch (y+)", data.getPitch());
        telemetry.addData("Imu Yaw   (z+)", data.getYaw());
    } 
}