package org.firstinspires.ftc.teamcode.teamprograms;

import java.util.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamprograms.auto.AutoInit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.io.Closeable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

@TeleOp(group = "AAA")
@Config
public class Robot2TestIntakeSimple extends LinearOpMode {

    Servo intakePivot;

    public static class ServoValues {
        public double pivotCarryPos = 0.23;
        public double pivotDepositPos = 0.172;
        public double pivotEjectSamplePos = 0.218;
        public double pivotIntakePos = 0.1825;
        public double pivotRestPos = 0.195;
    }
    public static ServoValues SERVO_VALUES = new ServoValues();

    public void runOpMode() {


        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad2.a) {
                intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);
            }

            if (gamepad2.b) {
                intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
            }

            if (gamepad2.y) {
                intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
            }

    
            if(gamepad2.x) {
                intakePivot.setPosition(SERVO_VALUES.pivotDepositPos);
            }

        }

    }
}