package org.firstinspires.ftc.teamcode.teamprograms.auto;

import org.firstinspires.ftc.teamcode.teamprograms.ButtonPressHandler;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

/**
 * Takes a preloaded sample and pushes it into the net zone. Grabs more neutral
 * samples afterwards.
 * 
 * @author Connor Larson
 */
@Autonomous(name="Sample Pusher", group="Auto")
public class SamplePusherAuto extends AutoCommonPaths {
    boolean isBlue = true;
    boolean shouldParkObservation = true; // False: Go to ascent zone there
    int neutralTagId = AprilLocater.NEUTRAL_BLUE_ID;
    int coloredTagId = AprilLocater.COLORED_BLUE_ID;

    ButtonPressHandler toggleBlueSide;
    ButtonPressHandler toggleObservationPark;
    ButtonPressHandler DEV_continue;

    final Pose2d START_LOCATION = new Pose2d(-63, 39, Math.toRadians(90));
    int DEV_step = 0;

    @Override
    public void init() {
        super.init();
        
        globalDrive = new MecanumDrive(hardwareMap, START_LOCATION);

        // Creating init_loop options
        try {
            toggleBlueSide = new ButtonPressHandler(gamepad1, "a", (Gamepad g) -> {
                isBlue = !isBlue;
                neutralTagId = isBlue ? AprilLocater.NEUTRAL_BLUE_ID : AprilLocater.NEUTRAL_RED_ID;
                coloredTagId = isBlue ? AprilLocater.COLORED_BLUE_ID : AprilLocater.COLORED_RED_ID;
            });

            toggleObservationPark = new ButtonPressHandler(gamepad1, "b", (Gamepad g) -> {
                shouldParkObservation = !shouldParkObservation;
            });
            
            DEV_continue = new ButtonPressHandler(gamepad1, "start", (Gamepad g) -> {
                DEV_step++;
            });
        } catch(NoSuchFieldException | NullPointerException err) {
            telemetry.addData("!!CAUGHT BUTTON ERROR", err);
        }
    }

    @Override 
    public void init_loop() {
        telemetry.addLine("==== POSITIONING NOTES ====");
        telemetry.addLine(
            "The robot should be positioned on the close edge of the tile" +
            "closest to the net zone (baskets area) without being inside it. The" + 
            "robot should be against the wall with the front facing the net zone. "
        );
        telemetry.addLine("Thank you!!! ❤️🦾");

        telemetry.addLine("");
        telemetry.addLine("==== OPTION CURRENT VALUES ====");
        telemetry.addData("Is Blue Side", isBlue);
        telemetry.addData("Should Park Observation", shouldParkObservation);


        telemetry.addLine("");
        telemetry.addLine("==== OPTION CONTROLS ====");
        telemetry.addData("Toggle Blue Side", "Press " + toggleBlueSide.getButtonName());
        telemetry.addData("Toggle Observation Park", "Press B" + toggleObservationPark.getButtonName());

        // Detecting buttonPresses
        try {
            toggleBlueSide.activateIfPressed();
            toggleObservationPark.activateIfPressed();
        } catch(NoSuchFieldException | IllegalAccessException err) {
            telemetry.addData("!!CAUGHT BUTTON ERROR", err);
        }
    }

    private void DEV_awaitContinue() {
        final int startStep = DEV_step;

        while(startStep == DEV_step) {
             try {
                DEV_continue.activateIfPressed();
            } catch(Exception err) {
                telemetry.addData("!!CAUGHT ERROR", err);
                telemetry.update();

                try {
                    Thread.sleep(3000) ;
                } catch (InterruptedException err2) {
                    telemetry.addData("!!CAUGHT ERROR", err2);
                    telemetry.update();
                }
        
                requestOpModeStop();
            }
        }
    }

    @Override
    public void start() {
        // Completely disabling the idea of hitting buttons
        toggleBlueSide = null;
        toggleObservationPark = null;

        // Driving to the net zone
        // final AprilTagDetection netZoneInitial = getDetection(this.neutralTagId);
        // moveRobotToNetZone(netZoneInitial); // TODO: do this based on the tag
        globalDrive.updatePoseEstimate();
        moveRobotToNetZone(isBlue);

        // Driving to the spike marks
        // DEV_awaitContinue();

        for(int i = 2; i >= 0; i--) {
            final AprilTagDetection spikeMark = getDetection(this.neutralTagId);
            globalDrive.updatePoseEstimate();
            moveRobotToSpikeMark(spikeMark, i, this.neutralTagId);
            globalDrive.updatePoseEstimate();
            moveRobotToNetZone(isBlue);
        }

        // Parking
        // DEV_awaitContinue();

        telemetry.addData("Status", "Completed!");
        telemetry.update();

        final AprilTagDetection observationZone = getDetection(this.coloredTagId);
        if(this.shouldParkObservation) {
            globalDrive.updatePoseEstimate();
            moveRobotToObservation(observationZone, /* ReverseAfter: */ false); 
        } else {
            globalDrive.updatePoseEstimate();
            moveRobotToAscentZone(observationZone);
            globalDrive.updatePoseEstimate();
            attemptAscent(1);
        }

        // Yipee! Finishing up
        telemetry.addData("Status", "Completed! 🥳");
        telemetry.update();

        try {
            Thread.sleep(3000) ;
        } catch (InterruptedException err) {
            telemetry.addData("!!CAUGHT ERROR", err);
            telemetry.update();
        }

        requestOpModeStop();
    }
}
