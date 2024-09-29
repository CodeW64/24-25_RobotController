package org.firstinspires.ftc.teamcode.teamprograms.auto;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Auto;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

/**
 * Implements utility methods and fields useful for creating autos that follow 
 * paths. This includes global location using april tags and odometry, common
 * autonomous routines (parking, e.g.)
 */
public class AutoPathFollower extends AprilLocater {
    private Pose2d currentPostion = new Pose2d(0, 0, 0);

    public static COLORED_RED_ID = 11;
    public static NEUTRAL_BLUE_ID = 13;
    public static COLORED_BLUE_ID = 14;
    public static NEUTRAL_RED_ID = 16;

    public static SPIKE_DISTANCE = 10; // In inches 
    

    protected double getX() {
        return currentPostion.position.x;
    }
    
    protected double getY() {
        return currentPostion.position.y;
    }

    protected double getTurn() {
        return currentPosition.heading.toDouble();
    }

    protected void setPosition(Pose2d newPose) {
        currentPosition = newPose;
    }

    
}