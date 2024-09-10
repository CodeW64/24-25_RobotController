package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;
// import com.acmerobotics.roadrunner.PIDCoefficients;
// import com.acmerobotics.roadrunner.PIDContoller;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;



@TeleOp(name = "LarsonTest")
public class LarsonTest extends PushBot {
  @Override
  public void init() {
    initPushBot();
    drive = new MecanumDrive(hardwareMap, startPose);
  }

  final Pose2d startPose = new Pose2d(0, 0, 0);
  boolean isStopped = false;
  
  // Tell the system what our robot is and where it starts
  MecanumDrive drive;
  final int RADIUS = 36;

  @Override 
  public void start() {
  } 

  @Override
  public void loop() {
    // Move in a circle
    Action moveInCircle = drive.actionBuilder(startPose)
      // .setTangent(0)
      .splineTo(new Vector2d(RADIUS, RADIUS), Math.toRadians(90))
      .splineTo(new Vector2d(0,  2 * RADIUS), Math.toRadians(180))
      .splineTo(new Vector2d(-RADIUS,RADIUS), Math.toRadians(270))
      .splineTo(new Vector2d(0,  0), Math.toRadians(360))
      .build();

    Actions.runBlocking(moveInCircle);
    
    telemetry.addData("Is pressing a", gamepad1.a);
    telemetry.update();
    if(gamepad1.a) {
      requestOpModeStop();
    }
  }

  @Override 
  public void stop() {
    isStopped = true;
  }
}