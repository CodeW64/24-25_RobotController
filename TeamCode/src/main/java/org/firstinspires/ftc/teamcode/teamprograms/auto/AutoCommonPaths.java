package org.firstinspires.ftc.teamcode.teamprograms.auto;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

/**
 * Implements utility methods and fields useful for creating autos that follow 
 * paths. 
 */
public class AutoCommonPaths extends AprilLocater {
    // private MecanumDrive mecDrive;

    // @Override
    // public void init() {
    //     super.init();
    //     mecDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    // }

    /**
     * The coordinate difference between the center of the robot and the camera
     * lense. If the camera is forward, the x-cordinate is negative. If the 
     * camera is to the right, the y-coordinate is positive. 
     */
    private Pose2d CAMERA_LENS_OFFSET = new Pose2d(0, 0, 0);

    public Pose2d getPointAwayFromTag(AprilTagDetection tag, double distBack, double rightStrafe) {
        // Getting useful information about where the TAG is. Not necessarily where to go.
        final double drive = tag.ftcPose.range;                       // Distance from the tag
        final double strafe = Math.toRadians(tag.ftcPose.bearing);    // Angle robot is way from tag
        final double turn = Math.toRadians(tag.ftcPose.yaw + 90); // Angle of the tag
        
        final double tagCenterX = tag.ftcPose.x;
        final double tagCenterY = tag.ftcPose.y;

        // Finding the point that the robot will drive to
        final double xBack = distBack * Math.cos(turn + Math.PI / 2);
        final double yBack = distBack * Math.sin(turn + Math.PI / 2);
        final double xStrafe = rightStrafe * Math.cos(turn + Math.PI); // Strafe to the right, x
        final double yStrafe = rightStrafe * Math.sin(turn + Math.PI); // Strafe to the right, y

        // The point is perpendicular to the April tag's turn, so go distBack at turn + 90deg away.
        // Plus and minus exist becuase two points exist so far away from the tag, but only one makes sense.
        final double plusDesiredX = tagCenterX + xBack + xStrafe;
        final double plusDesiredY = tagCenterY + yBack + yStrafe;
        final double minusDesiredX = tagCenterX - xBack - xStrafe;
        final double minusDesiredY = tagCenterY - yBack - yStrafe;
        
        // Defaulting to minus the perpendicular values. If the closer one is plus, that's handled by the if()
        double destinationX = minusDesiredX;
        double destinationY = minusDesiredY;
        double destinationTurn = turn - Math.PI / 2;

        // Choosing plus if it is closer instead of minus.
        if(
            plusDesiredX * plusDesiredX + plusDesiredY * plusDesiredY <
            minusDesiredX * minusDesiredX + minusDesiredY * minusDesiredY
        ) {
            destinationX = plusDesiredX;
            destinationY = plusDesiredY;
            destinationTurn = turn + Math.PI / 2;
        }

        return new Pose2d(destinationX, destinationY, destinationTurn);
    }

    /**
     * Moves the robot into a position to place a specimen on a chamber
     * 
     * @param tag AprilTagDetection - The detection of the nearest spike 
     *     mark. 
     * @param distFrom double - The desired distance from the chamber
     */
    public void moveRobotToChamber(AprilTagDetection tag, double distFrom) {
        // Finding the distance directly away from the April Tag
        /* Diagram of the distance:
         *
         *          \   Tag
         *          /\   * 
         *    curDis    /|    ___
         *      /     / (|     |
         *   \/     /    |  desiredDist
         *    \   /_____[|    _|_
         *      *        * 
         *    Robot     Chamber
         * 
         * (The angle annotated with the paranthesis is the bearing)
         * You can use cos(x) to find the desired dist. Note that the robot must 
         * strafe farther because the April tag does not line up perfectly
         */
        final double curDist = tag.ftcPose.range;
        final double bearing = tag.ftcPose.bearing;
        final double desiredDist = curDist * Math.cos(bearing);

        // Getting the offset right from the April tag to the chambers
        // Getting the offset right from the tag to the spike marks' centers
        final double TILE_SIZE = 24; // In inches
        double centerOffset = TILE_SIZE - distFrom; // inches

        final boolean isNeutralMark = 
            tag.id == AprilLocater.NEUTRAL_RED_ID || 
            tag.id == AprilLocater.NEUTRAL_BLUE_ID;
        if(!isNeutralMark) {
            // If the tag is colored, then field setup means that the tag is to the left (-)
            centerOffset *= -1;
        }

        // Getting the and going to it in a line
        final Pose2d destination = getPointAwayFromTag(tag, desiredDist, centerOffset);
        
        // DEV START: this checks to make sure that the distance is actually approx the estimated distance
        final double ACTUAL_DIST = 2 * TILE_SIZE - 18;
        final double TOLERANCE = 3; // inches
        if(Math.abs(Math.hypot(destination.position.x, destination.position.y) - ACTUAL_DIST) < TOLERANCE) {

        }
        // DEV End
        
        telemetry.addData("Status", "Executing action...");
        telemetry.update();

        final MecanumDrive mecDrive = new MecanumDrive(hardwareMap, CAMERA_LENS_OFFSET);
        Action splineToTag = mecDrive.actionBuilder(new Pose2d(0, 0, 0))
            // .lineToLinearHeading(destination)
            .setTangent(Math.atan2(destination.position.y, destination.position.x))
            .lineToXLinearHeading(destination.position.x, destination.heading.toDouble())
            .build();
        Actions.runBlocking(splineToTag);
        telemetry.addData("Status", "Finished action!");
        
    }

    /**
     * Moves the robot to the given spike mark away from the given April tag
     * 
     * @param tag AprilTagDetection - Position data for the corresponding April Tag
     * @param markNum int - The 0-based index of the mark away from the wall. 0 
     *     For the spike mark closest to the wall, 1 for middle, 2 for the 
     *     farthest.
     */
    public void moveRobotToSpikeMark(AprilTagDetection tag, int tagNum) {
        // Getting the offset right from the tag to the spike marks' centers
        final double HALF_TAG_LENGTH = 1.75; // inches
        final double TILE_SIZE = 24; // In inches
        double centerOffset = TILE_SIZE - HALF_TAG_LENGTH; // inches

        final boolean isNeutralMark = 
            tag.id == AprilLocater.NEUTRAL_RED_ID || 
            tag.id == AprilLocater.NEUTRAL_BLUE_ID;
        if(!isNeutralMark) {
            // If the tag is colored, then field setup means that the tag is to the left (-)
            centerOffset *= -1;
        }

        // Getting the center point of the spike mark (where the sample should be).
        final double EACH_SPIKE_DIST = 10; // In inches
        final double ZERO_DIST = 2;        // Spike 0's distance from wall, in inches
        final double distFrom = EACH_SPIKE_DIST * tagNum + ZERO_DIST;
        final Pose2d destination = getPointAwayFromTag(tag, distFrom, centerOffset);

        // Moving linearly to the spike mark's center.
        telemetry.addData("Status", "Executing action...");
        telemetry.update();

        final MecanumDrive mecDrive = new MecanumDrive(hardwareMap, CAMERA_LENS_OFFSET);
        Action splineToTag = mecDrive.actionBuilder(new Pose2d(0, 0, 0))
            // .lineToLinearHeading(destination)
            .setTangent(Math.atan2(destination.position.y, destination.position.x))
            .lineToXLinearHeading(destination.position.x, destination.heading.toDouble())
            .build();
        Actions.runBlocking(splineToTag);
        telemetry.addData("Status", "Finished action!");
    }
}