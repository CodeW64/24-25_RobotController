package org.firstinspires.ftc.teamcode.teamprograms.auto;

import java.util.Vector;

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
    protected MecanumDrive globalDrive;

    @Override
    public void init() {
        super.init();
        globalDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    /**
     * The coordinate difference between the center of the robot and the camera
     * lense. If the camera is forward, the x-cordinate is negative. If the 
     * camera is to the right, the y-coordinate is positive. 
     */
    private Pose2d CAMERA_LENS_OFFSET = new Pose2d(0, 0, 0);

    private Pose2d getCurrentPosition() {
        // Getting the current position of the robot
        globalDrive.updatePoseEstimate();
        return globalDrive.pose;
    }

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
    protected void moveRobotInitalToChamber(AprilTagDetection tag, double distFrom) {
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
        final MecanumDrive mecDrive = new MecanumDrive(hardwareMap, CAMERA_LENS_OFFSET);
        
        // Checking that the distance to travel is approz what it should be
        final double ACTUAL_DIST = 2 * TILE_SIZE - 18;
        final double TOLERANCE = 3; // inches
        final double foundDist = Math.hypot(destination.position.x, destination.position.y);
        if(Math.abs(foundDist - ACTUAL_DIST) > TOLERANCE) {
            telemetry.addData("!!WARNING", String.format("Distance between the April dest and hypot is %d", foundDist));
            // The destination seems very wrong; execute fallback path (moves forward);
            telemetry.addData("Status", "executing fallback path to chambers");
            telemetry.update();

            Action fallbackLineToDestionation = mecDrive.actionBuilder(new Pose2d(0, 0, 0))
                // .setTangent(0)
                .lineToX(ACTUAL_DIST)
                .build();
            return;
        }
        
        // Executing the action!!
        telemetry.addData("Status", "Executing action...");
        telemetry.update();

        Action lineToDestionation = mecDrive.actionBuilder(new Pose2d(0, 0, 0))
            // .lineToLinearHeading(destination)
            .setTangent(Math.atan2(destination.position.y, destination.position.x))
            .lineToXLinearHeading(destination.position.x, destination.heading.toDouble())
            .build();
        Actions.runBlocking(lineToDestionation);
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
    protected void moveRobotToSpikeMark(AprilTagDetection tag, int markNum) {
        // Getting the offset right from the tag to the spike marks' centers
        final double HALF_TAG_LENGTH = 1.75; // inches
        final double TILE_SIZE = 24; // In inches
        final double EACH_SPIKE_DIST = 10; // In inches
        final double ZERO_DIST = 2;        // Spike 0's distance from wall, in inches
        final double distFrom = EACH_SPIKE_DIST * markNum + ZERO_DIST;

        // Executing a fallback if the tag is null
        if(tag == null) {
            telemetry.addData("Status", "executing fallback path to spike mark " + markNum);
            telemetry.update();

            // Move to the spike mark using only odometry
            final Vector2d spike0Pos = new Vector2d(TILE_SIZE, TILE_SIZE);
            lineTo(spike0Pos.minus(new Vector2d(0, distFrom)));
            return;
        }

        // Getting the offset from the edge to the center of the spike mark
        double centerOffset = TILE_SIZE - HALF_TAG_LENGTH; // inches
        final boolean isNeutralMark = 
            tag.id == AprilLocater.NEUTRAL_RED_ID || 
            tag.id == AprilLocater.NEUTRAL_BLUE_ID;
        if(!isNeutralMark) {
            // If the tag is colored, then field setup means that the tag is to the left (-)
            centerOffset *= -1;
        }

        // Getting the center point of the spike mark (where the sample should be).;
        final Pose2d destination = getPointAwayFromTag(tag, distFrom, centerOffset);


        // Moving linearly to the spike mark's center.
        telemetry.addData("Status", "Executing action...");
        telemetry.update();

        final MecanumDrive mecDrive = new MecanumDrive(hardwareMap, CAMERA_LENS_OFFSET);
        Action lineToDestionation = mecDrive.actionBuilder(new Pose2d(0, 0, 0))
            // .lineToLinearHeading(destination)
            .setTangent(Math.atan2(destination.position.y, destination.position.x))
            .lineToXLinearHeading(destination.position.x, destination.heading.toDouble())
            .build();
        Actions.runBlocking(lineToDestionation);
        telemetry.addData("Status", "Finished action!");
    }

    /**
     * Moves the robot to the observation zone given the april tag
     * 
     * @param tag AprilTagDetection - Position data for the corresponding April Tag
     */
    protected void moveRobotToObservation(AprilTagDetection tag, boolean reverseAfter) {

    }

    /**
     * Moves the robot to the netZone given the april tag
     * 
     * @param tag AprilTagDetection - Position data for the corresponding April Tag
     */
    protected void moveRobotToNetZone(AprilTagDetection tag) {
        // TODO: Fill this with the needed code to run based on the tag.
    }

    /**
     * Moves the robot to the netZone based solely on odometry
     */
    protected void moveRobotToNetZone() {
        // Moving the robot forward based on the odometry
        final int TILE_SIZE = 24; // In inches
        final Vector2d netZonePosition = new Vector2d(0, TILE_SIZE / 2);
        lineTo(netZonePosition);
    }

    private void lineToLinearHeading(Pose2d target) {
        // Moving the robot forward based on the odometry
        final Vector2d currentPos = getCurrentPosition().position;
        final Vector2d deltaPosition = target.position.minus(currentPos);
        Action moveToTarget = globalDrive.actionBuilder(new Pose2d(0, 0, 0))
            .setTangent(Math.atan2(deltaPosition.y, deltaPosition.x))    
            .lineToXLinearHeading(target.position.x, target.heading)
            .build();
        Actions.runBlocking(moveToTarget); // Pray 🤞
    }
    
    private void lineTo(Vector2d target) {
        // Moving the robot forward based on the odometry
        final Vector2d currentPos = getCurrentPosition().position;
        final Vector2d deltaPosition = target.minus(currentPos);
        Action moveToTarget = globalDrive.actionBuilder(new Pose2d(0, 0, 0))
            .setTangent(Math.atan2(deltaPosition.y, deltaPosition.x))    
            .lineToX(target.x)
            .build();
        Actions.runBlocking(moveToTarget); // Pray 🤞
    }

    /**
     * Moves the robot to the ascent zone given the april tag
     * 
     * @param tag AprilTagDetection - Position data for the corresponding April Tag
     */
    protected void moveRobotToAscentZone(AprilTagDetection tag) {
        // TODO: Fill this with the needed code to run based on the tag.


    }

    protected void attemptAscent(int levelAscent) {
        // TODO: Fill this

    }
}