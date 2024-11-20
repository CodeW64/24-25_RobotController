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
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

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
abstract public class AutoCommonPaths extends AprilLocater {
    protected MecanumDrive globalDrive;

    public final Vector2d FEILD_CENTER = new Vector2d(0, 0); // Origin

    public final static Pose2d BLUE_SIDE_TAG = new Pose2d(-72, 0, Math.toRadians(180 - 1e-10)); 
    public final static Pose2d BLUE_NET = new Pose2d(-60, 60, Math.toRadians(135)); 
    public final static Pose2d BLUE_OBSERVATION = new Pose2d(-60, 48, Math.toRadians(-135)); 
    public final static Pose2d BLUE_NEUTRAL_TAG = new Pose2d(-48, 72, Math.toRadians(90)); 
    public final static Pose2d BLUE_COLORED_TAG = new Pose2d(-48, -72, Math.toRadians(-90));
    public final static Pose2d BLUE_CHAMBER = new Pose2d(-24, 0, Math.toRadians(0));

    public final static Pose2d RED_SIDE_TAG = AutoCommonPaths.bluePoseToRed(AutoCommonPaths.BLUE_SIDE_TAG); 
    public final static Pose2d RED_NET = AutoCommonPaths.bluePoseToRed(AutoCommonPaths.BLUE_NET); 
    public final static Pose2d RED_OBSERVATION = AutoCommonPaths.bluePoseToRed(AutoCommonPaths.BLUE_OBSERVATION); 
    public final static Pose2d RED_NEUTRAL_TAG = AutoCommonPaths.bluePoseToRed(AutoCommonPaths.BLUE_NEUTRAL_TAG); 
    public final static Pose2d RED_COLORED_TAG = AutoCommonPaths.bluePoseToRed(AutoCommonPaths.BLUE_COLORED_TAG); 
    public final static Pose2d RED_CHAMBER = AutoCommonPaths.bluePoseToRed(AutoCommonPaths.BLUE_CHAMBER);

    public enum RotationType {
        LINEAR,
        SPLINE
    }

    private RotationType rotationType = RotationType.LINEAR;

    private Pose2d destinationOffset = new Pose2d(0, 0, 0);

    /**
     * Converts positions of blue side objects into a form usable on the red side.
     * 
     * @param bluePose2d A blue side object's position.
     * @return The red-usable form of the corresponding blue object.
     */
    public static Pose2d bluePoseToRed(Pose2d bluePose2d) {
        // return multiplyPose(bluePose2d, -1);
        return bluePose2d; // DEV NOTE: This should be changed if there is a relative differnece between red and blue.
    }

    /**
     * Returns the global Pose2d of the given April tag's center.
     * 
     * @param id The id of the Aprial Tag desired
     * @return The pose representing the center of the april tag.
     */
    public static Pose2d getTagPoseFromId(int id) {
        switch(id) {
            case AprilLocater.COLORED_BLUE_ID:
                return AutoCommonPaths.BLUE_COLORED_TAG;

            case AprilLocater.NEUTRAL_BLUE_ID:
                return AutoCommonPaths.BLUE_NEUTRAL_TAG;

            case AprilLocater.COLORED_RED_ID:
                return AutoCommonPaths.RED_COLORED_TAG;

            case AprilLocater.NEUTRAL_RED_ID:
            default:
                return AutoCommonPaths.RED_NEUTRAL_TAG;
        }    
    }

    @Override
    public void opMode_init() {
        super.opMode_init();
    }

    /**
     * Adds the given pose component-wise to the next "moveTo" commands. For 
     * example, if this was passed in Pose2d(23, -9, Math.PI / 3), and the robot
     * is later told to move to Pose2d(10, 10, 0), the robot will end up at 
     * Pose2d(33, 1, Math.PI / 3).  
     * 
     * <p> Multiple calls of this method will cumulate, adding together component 
     * wise.
     *  
     * @param additiveOffset The pose that is added component-wise to any 
     *     destination pose.
     */
    protected void addDestinationOffset(Pose2d offset) {
        destinationOffset = addPoses(destinationOffset, offset);
        telemetry.addLine("");
        telemetry.addData("Offset", logPose(offset));
        telemetry.addData("New Destination Offset", logPose(destinationOffset));
        telemetry.update();
    }

    public String logPose(Pose2d p) {
        return String.format(
            "(%f2, %f2) at %f2",
            p.position.x, 
            p.position.y,
            Math.toDegrees(p.heading.toDouble())
        );
    }

    /**
     * Adds the given pose component-wise to the next "moveTo" commands. For 
     * example, if this was passed in Pose2d(23, -9, Math.PI / 3), and the robot
     * is later told to move to Pose2d(10, 10, 0), the robot will end up at 
     * Pose2d(33, 1, Math.PI / 3).  
     * 
     * <p> Calls of this method will override any previous offset.
     *  
     * @param newOffset The pose that is added component-wise to any 
     *     destination pose.
     */
    protected void setDestinationOffset(Pose2d newOffset) {
        destinationOffset = newOffset;
        telemetry.addLine("");
        telemetry.addData("Offset", logPose(newOffset));
        telemetry.addData("New Destination Offset", logPose(destinationOffset));
        telemetry.update();
    }

    /**
     * Sets the rotationType attribute to the given value.
     * 
     * @param type The RotationType enum to input
     */
    protected void setRotationType(RotationType type) {
        rotationType = type;
    }

    /**
     * Clears all cumulative offset created by the addDestinationOffset method. 
     * All destinations are traveled directly to.
     */
    protected void resetDestinationOffset() {
        destinationOffset = new Pose2d(0, 0, 0);
    }

    /**
     * Creates a new Pose2d as a result of adding component-wise. That is, the
     * new Pose2d can be written as follows: new 
     * Pose2d(x1 + x2, y1 + y2, theta1 + theta2)
     * 
     * @param addend1 A Pose2d to be added with another.
     * @param addend2 A Pose2d to be added with another.
     * @return The component-wise sum of the poses.
     */
    public Pose2d addPoses(Pose2d addend1, Pose2d addend2) {
        return new Pose2d(
            addend1.position.plus(addend2.position),
            Rotation2d.fromDouble(addend1.heading.toDouble() + addend2.heading.toDouble())
        );
    }

    /**
     * Creates a new Pose2d as a result of subtracting component-wise. That is, 
     * the new Pose2d can be written as follows: new 
     * Pose2d(x1 - x2, y1 - y2, theta1 - theta2)
     * 
     * @param minuend The left-hand Pose2d, to be subtracted by another. 
     * @param subtrahend The right-hand Pose2d, to be subtracted from another.
     * @return The component-wise difference of the poses.
     */
    public Pose2d subtractPoses(Pose2d minuend, Pose2d subtrahend) {
        return new Pose2d(
            minuend.position.minus(subtrahend.position),
            Rotation2d.fromDouble(minuend.heading.toDouble() - subtrahend.heading.toDouble())
        ); 
    }

    /**
     * Creates a new Pose2d as a result of multiplying component-wise. That is, 
     * the new Pose2d can we written as new 
     * Pose2d(x * scalar, y * scalar, theta * scalar,)
     * 
     * @param poseFactor The pose to multiply by a scalar component-wise
     * @param scalar The number each componet of the pose is multiplied by
     * @return The component-wise, scalar multiplication of pose.
     */
    public Pose2d multiplyPose(Pose2d poseFactor, double scalar) {
        return new Pose2d(
            poseFactor.position.x * scalar,
            poseFactor.position.y * scalar,
            poseFactor.heading.toDouble() * scalar
        );
    }

    /**
     * Creates a new Pose2d as a result of multiplying component-wise. That is, 
     * the new Pose2d can we written as new 
     * Pose2d(x1 * x2, y1 * y2, theta1 * theta2)
     * 
     * @param factor1 A pose to multiply by another, component-wise
     * @param factor2 A pose to multiply by another, component-wise
     * @return The component-wise product of two poses.
     */
    public Pose2d multiplyPoses(Pose2d factor1, Pose2d factor2) {
        return new Pose2d(
            factor1.position.x * factor2.position.x,
            factor1.position.y * factor2.position.y,
            factor1.heading.toDouble() * factor2.heading.toDouble()
        );
    }

    /**
     * Transforms the given vector by a matrix. 
     * 
     * @param vec The vector to transform.
     * @param matrix A length-two array of 2d vectors. The first vector is 
     *     x-base, and the second is the y-base.
     * @return The matrix product of a matrix with a x-basis vector and a 
     *     y-basis vector, and the (column) vector provided
     */
    public Vector2d transformVector(Vector2d vec, Vector2d[] matrix) {
        return new Vector2d(
            vec.x * matrix[0].x + vec.y * matrix[1].y,
            vec.x * matrix[0].y + vec.y * matrix[1].x
        );
    }

    /**
     * Returns the current position of the global drive. The pose estimate is 
     * updated as a result.
     * 
     * @return The pose property of the global drive after calling 
     *     updatePoseEstimate
     */
    protected Pose2d getCurrentPosition() {
        // Getting the current position of the robot
        globalDrive.updatePoseEstimate();
        return globalDrive.pose;
    }

    /**
     * Returns the action builder of the drive at the given position, offset by 
     * destination offset.
     * 
     * @param drive The RR drivetrain to get the builder of
     * @param position The starting position of the drive
     * @return The drive's Actionbuilder at the given position, offset.
     */
    private TrajectoryActionBuilder actionBuilder(MecanumDrive drive, Pose2d position) {
        // Rotating the offset. If this is not done, then the axes are messed up,
        // and the robot will drive to the wrong spot if the heading is offset.
        // final double headingCos = destinationOffset.heading.real;
        // final double headingSin = destinationOffset.heading.imag;
        // final Vector2d[] rotationMatrix = {
        //     new Vector2d(  headingCos, headingSin ),
        //     new Vector2d( -headingSin, headingCos )
        // };
        // final Vector2d rotatedVector = transformVector(destinationOffset.position, rotationMatrix);
        // final Pose2d rotatedOffset = new Pose2d(rotatedVector, destinationOffset.heading);

        // telemetry.addLine("");
        // telemetry.addData("RotatedOffset", logPose(rotatedOffset));
        // telemetry.update();

        // // Getting you one fresh, fine offset actionbuilder!
        // return drive.actionBuilder(addPoses(position, rotatedOffset));
        return drive.actionBuilder(position);
    }

    /**
     * Finds the point some distance out front of the given april tag.
     * 
     * @param tag The tag to park away from
     * @param distBack How far away, in inches, to park directly away from.
     * @param rightStrafe How far away, in inches, the robot should go right 
     *     from the point precisely distBack from the tag.
     * @return A pose with the properties:
     *  <ol><li> Distance from the tag's center is hypot(distBack, rightStrafe)
     *      <li> The rotation is perpendicular to the tag.
     *      <li> Strafing rightStrafe distance left cancels out the original 
     *           strafe offset.
     */
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
     * Moves the robot into a position to place a specimen on a chamber.
     */
    protected void moveToCloseChamberInitial() {
        lineTo(globalDrive, BLUE_CHAMBER);
    }

    /**
     * Moves the robot forward the given distance
     * 
     * @param dist How far to move forward, in inches
     */
    protected void moveForward(double dist) {
        final Vector2d pos = getCurrentPosition().position;
        final double theta = getCurrentPosition().heading.toDouble();
        final double newX = + dist * Math.cos(theta);
        final double newY = + dist * Math.sin(theta);
        
    }

    /**
     * Moves the robot to the given spike mark away from the given April tag. 
     * 
     * @param tag AprilTagDetection - Position data for the corresponding April Tag
     * @param markNum int - The 0-based index of the mark away from the wall. 0 
     *     For the spike mark closest to the wall, 1 for middle, 2 for the 
     *     farthest.
     */
    protected void moveRobotToSpikeMark(AprilTagDetection tag, int markNum, int fallBackId) {
        // Getting the offset right from the tag to the spike marks' centers
        final double HALF_TAG_LENGTH = 1.75; // inches
        final double TILE_SIZE = 24;         // In inches
        final double EACH_SPIKE_DIST = 10;   // In inches
        final double ZERO_DIST = 4;          // Spike 0's distance from wall, in inches
        final double distFrom = EACH_SPIKE_DIST * markNum + ZERO_DIST;

        // Getting the offset from the edge to the center of the spike mark
        double centerOffset = TILE_SIZE - HALF_TAG_LENGTH; // inches to the right
        final boolean isNeutralMark = 
            fallBackId == AprilLocater.NEUTRAL_RED_ID || 
            fallBackId == AprilLocater.NEUTRAL_BLUE_ID;
        if(!isNeutralMark) {
            // If the tag is colored, then field setup means that the tag is to the left (-)
            centerOffset *= -1;
        }

        // Executing a fallback if the tag is null
        if(tag == null) {
            telemetry.addData("Status", "executing fallback path to spike mark " + markNum);
            telemetry.update();

            // Move to the spike mark using only odometry
            final Pose2d spikeTag = getTagPoseFromId(fallBackId);
            lineTo(globalDrive, new Pose2d(
                spikeTag.position.x + centerOffset + 2,
                spikeTag.position.y - distFrom * Math.signum(centerOffset),
                // spikeTag.heading.toDouble()
                Math.toRadians(90)
            ));
            return;
        }

        // Getting the center point of the spike mark (where the sample should be).;
        final Pose2d targetPoint = getPointAwayFromTag(tag, distFrom, centerOffset);
        final Pose2d destination = addPoses(targetPoint, destinationOffset);

        // Moving linearly to the spike mark's center.
        telemetry.addData("Status", "Executing action...");
        telemetry.update();

        final MecanumDrive mecDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
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
     * 
     * @param isBlue Whether the robot is on the blue side. This determines 
     *     which global coordinates to use.
     */
    protected void moveRobotToNetZone(boolean isBlue) {
        // Moving the robot forward based on the odometry
        final int TILE_SIZE = 24; // In inches
        Pose2d pose = pose = AutoCommonPaths.RED_NET;
        if(isBlue) {
            pose = AutoCommonPaths.BLUE_NET;
        }
        lineTo(globalDrive, pose);
    }

    /**
     * Moves the robot to the net zone based soley on odometry. This is done 
     * relative to the start point, so no distinction is made between red or 
     * blue.
     */
    protected void moveRobotToNetZone() {
        moveRobotToNetZone(true);
    }
    
    /**
     * Uses RR to move in a line to the given point. the heading changes 
     * along a spline curve while moving, resulting at the given target's 
     * heading. 
     * 
     * @param drive The RR mecanum drivetrain to use as a basis. 
     * @param target The destination.
     */
    protected void lineToSplineHeading(MecanumDrive drive, Pose2d target) {
        // Moving the robot forward based on the odometry
        final Pose2d currentPos = getCurrentPosition();
        final Pose2d offsetTarget = addPoses(target, destinationOffset);
        final Vector2d deltaPosition = offsetTarget.position.minus(currentPos.position);
        Action moveToTarget;
        if(deltaPosition.x == 0) {
            moveToTarget = drive.actionBuilder(currentPos)
                .setTangent(Math.atan2(deltaPosition.y, deltaPosition.x))    
                .lineToYSplineHeading(offsetTarget.position.y, offsetTarget.heading)
                .build();

        } else {
            moveToTarget = drive.actionBuilder(currentPos)
                .setTangent(Math.atan2(deltaPosition.y, deltaPosition.x))    
                .lineToXSplineHeading(offsetTarget.position.x, offsetTarget.heading)
                .build();

        }
        Actions.runBlocking(moveToTarget); // Pray 🤞
    }
 

    /**
     * Uses RR to move in a line to the given point. the heading changes 
     * linearly while moving, resulting at the given target's heading. 
     * 
     * @param drive The RR mecanum drivetrain to use as a basis. 
     * @param target The destination.
     */
    protected void lineToLinearHeading(MecanumDrive drive, Pose2d target) {
        // Moving the robot forward based on the odometry
        final Pose2d currentPos = getCurrentPosition();
        final Pose2d offsetTarget = addPoses(target, destinationOffset);
        final Vector2d deltaPosition = offsetTarget.position.minus(currentPos.position);
        Action moveToTarget;
        if(deltaPosition.x == 0) {
            moveToTarget = drive.actionBuilder(currentPos)
                .setTangent(Math.atan2(deltaPosition.y, deltaPosition.x))    
                .lineToYLinearHeading(offsetTarget.position.y, offsetTarget.heading)
                .build();

        } else {
            moveToTarget = drive.actionBuilder(currentPos)
                .setTangent(Math.atan2(deltaPosition.y, deltaPosition.x))    
                .lineToXLinearHeading(offsetTarget.position.x, offsetTarget.heading)
                .build();

        }
        Actions.runBlocking(moveToTarget); // Pray 🤞
    }
 
    /**
     * Uses RR to move in a line to the given point. The heading remainins the 
     * same after moving.
     * 
     * @param drive The RR mecanum drivetrain to use as a basis. 
     * @param target The destination.
     */
    protected void lineTo(MecanumDrive drive, Vector2d target) {
        // Moving the robot forward based on the odometry
        final Pose2d currentPos = getCurrentPosition();
        final Vector2d offsetTarget = target;
        final Vector2d deltaPosition = offsetTarget.minus(currentPos.position);
        Action moveToTarget;
        if(deltaPosition.x == 0) {
            moveToTarget = drive.actionBuilder(currentPos)
                .setTangent(Math.atan2(deltaPosition.y, deltaPosition.x))    
                .lineToY(offsetTarget.y)
                .build();

        } else {
            moveToTarget = drive.actionBuilder(currentPos)
                .setTangent(Math.atan2(deltaPosition.y, deltaPosition.x))    
                .lineToX(offsetTarget.x)
                .build();

        }
        Actions.runBlocking(moveToTarget); // Pray 🤞
    }

    /**
     * Uses RR to move in a line to the given point. the heading changes 
     * with the object's current rotationType, resulting at the given target's 
     * heading. 
     * 
     * @param drive The RR mecanum drivetrain to use as a basis. 
     * @param target The destination.
     */
    protected void lineTo(MecanumDrive drive, Pose2d target) {
        if(rotationType.equals(RotationType.LINEAR)) {
            lineToLinearHeading(drive, target);
        } else if(rotationType.equals(RotationType.SPLINE)) {
            lineToSplineHeading(drive, target);
        } else {
            throw new RuntimeException("Invalid rotationType attribute");
        }
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