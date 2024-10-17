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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
@Autonomous(name="Basket Placer (Preloaded Specimen)")
public class SamplePreloadBasket extends AutoCommonPaths {
    boolean isBlue = true;
    boolean shouldParkObservation = true; // False: Go to ascent zone there
    int neutralTagId = AprilLocater.NEUTRAL_BLUE_ID;
    int coloredTagId = AprilLocater.COLORED_BLUE_ID;
    boolean repositionEnabled = false;

    ButtonPressHandler toggleBlueSide;
    ButtonPressHandler toggleObservationPark;
    ButtonPressHandler repositionToggle;

    final Pose2d START_LOCATION = new Pose2d(-63, 43, Math.toRadians(90));

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
            
            repositionToggle = new ButtonPressHandler(gamepad1, "start", (Gamepad g) -> {
                repositionEnabled = !repositionEnabled;
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
        telemetry.addData("Reposition Enabled", repositionEnabled);

        telemetry.addLine("");
        telemetry.addLine("==== OPTION CONTROLS ====");
        telemetry.addData("Toggle Blue Side", "Press " + toggleBlueSide.getButtonName());
        telemetry.addData("Toggle Observation Park", "Press " + toggleObservationPark.getButtonName());
        telemetry.addData("Respoition Toggle", "Press " + repositionToggle.getButtonName());
        telemetry.addData("[[DEV]] Await Continue", "Press start");

        // Detecting buttonPresses
        try {
            toggleBlueSide.activateIfPressed();
            toggleObservationPark.activateIfPressed();
            repositionToggle.activateIfPressed();
        } catch(NoSuchFieldException | IllegalAccessException err) {
            telemetry.addData("!!CAUGHT BUTTON ERROR", err);
        }

        if(repositionEnabled) {
            driveWheels();
        }
    }

    @Override
    public void start() {
        telemetry.clearAll();
        telemetry.setAutoClear(false);

        // Completely disabling the idea of hitting buttons
        toggleBlueSide = null;
        toggleObservationPark = null;
        repositionToggle = null;

        // Driving to the net zone
        // final AprilTagDetection netZoneInitial = getDetection(this.neutralTagId);
        globalDrive.updatePoseEstimate();
        moveRobotToNetZone(isBlue);

        // Driving to the spike marks
        for(int i = 2; i >= 0; i--) {
            final AprilTagDetection spikeMark = getDetection(this.neutralTagId);
            final Pose2d robotFrontOffset = new Pose2d(0, -10, 0);

            // Moving to grab the sample
            setDestinationOffset(robotFrontOffset); // The front goes to the destination, not the robot center
            globalDrive.updatePoseEstimate();
            moveRobotToSpikeMark(spikeMark, i, this.neutralTagId);

            // Pushing into the pixel to grasp it
            resetDestinationOffset();
            final Pose2d currentPosition = getCurrentPosition();
            lineToLinearHeading(globalDrive, new Pose2d(
                currentPosition.position.x,
                currentPosition.position.y + 10,
                Math.atan2(
                    AutoCommonPaths.BLUE_NET.position.y - (currentPosition.position.y + 10),
                    AutoCommonPaths.BLUE_NET.position.x - currentPosition.position.x
                )
            ));
            
            // Scoring!
            globalDrive.updatePoseEstimate();
            moveRobotToNetZone(isBlue);
        }

        // Parking
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

    @Override
    public void stop() {
        powerDriveMotors(0, 0, 0, 0);
    }
}
