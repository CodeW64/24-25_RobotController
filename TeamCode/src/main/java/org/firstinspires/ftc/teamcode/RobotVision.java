package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * R.I.P. Tensorflow. You shall be most dearly missed in the Autonomous. <br> <br>
 * <strong>A Haiku dedicated to the memory of Tensorflow, written by ChatGPT</strong> <br>
 * Farewell, TensorFlow, <br>
 * Silent pixels pause their gaze, <br>
 * April Tags now lead. <br>
 */

@Config
public class RobotVision {


    public enum Direction {
        LEFT, RIGHT

    }

    /*public enum AllianceSpecific {

        RED_ALLIANCE("TeamPropRed"),
        BLUE_ALLIANCE("TeamPropBlue");


        private final String label;
        AllianceSpecific(String label) {
            this.label = label;
        }
    }*/


    /*private static final String TFOD_MODEL_ASSET = "CenterstageMeetTwo.tflite";
    private static final String[] LABELS = {
            "TeamPropBlue",
            "TeamPropRed",
    };*/

    private final String[] pathValues = {"Left", "Center", "Right"};


    // Gains for driving to an April Tag (editable by FTC dashboard)
    public static class AutoGains {
        public double driving = 0.035;
        public double strafe = 0.025;
        public double turning = 0.04; // 0.025
    }
    public static AutoGains AUTO_GAINS = new AutoGains();


    // Speed caps for driving to an April Tag (editable by FTC dashboard)
    public static class AutoSpeeds {
        public double maxDriving = 0.4;
        public double maxStrafe = 0.3;
        public double maxTurning = 0.5;

        public double correctionDriving = 0.2;
        public double correctionStrafe = 0.2;
        public double correctionTurning = 0.3;
    }
    public static AutoSpeeds AUTO_SPEEDS = new AutoSpeeds();




    // REFERENCE

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.

    //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0) | ORIGINAL 0.02
    //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0) | ORIGINAL 0.015
    //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0) | ORIGINAL 0.01

    private final DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag;

//    private TfodProcessor tfod;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;


    private final boolean usingAprilTag;
    private final boolean usingTensorflow;
    private boolean initialized;



    public void checkExceptionValidity() throws FailedInitializationException {
        throw new FailedInitializationException("Vision is not fully initialized; you failed to check for "+
                "\"isCameraInitialized()\" before calling \"setManualExposure()\"");
    }


    public RobotVision(HardwareMap hardwareMap, Telemetry telemetry, boolean usingAprilTag, boolean usingTensorflow) {

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.usingAprilTag = usingAprilTag;
        this.usingTensorflow = usingTensorflow;
        this.desiredTag = null;
        this.initialized = false;

        if (usingAprilTag) initAprilTag();
//        if (usingTensorflow) initTensorflow();
        initVisionPortal();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);


    }




    /*
     * @return path value for autonomous mode.
     */
    /*public int scanForTensorflowRecognitions(AllianceSpecific alliance) {

        double posX = 0;
        int path = 10;

        String modelType = null;

        int modelHeight = 0;
        int modelConfidence = 0;

        int incorrectProps = 0;


        List<Recognition> currentRecognitions = tfod.getRecognitions();
        for (Recognition recognition : currentRecognitions) {

            modelConfidence = (int)recognition.getConfidence();
            modelType = recognition.getLabel();

            // if it detects the wrong type prop, skip over
            if (!modelType.equals(alliance.label)) {
                incorrectProps++;
                continue;
            }


            modelHeight = (int)recognition.getHeight();

            // if it detects something larger than normal size, skip
            if (modelHeight < 140 || modelHeight > 300) {
                incorrectProps++;
                continue;
            }

            posX = (recognition.getLeft() + recognition.getRight()) / 2 ;



        }

        telemetry.addData("# Objects Detected", currentRecognitions.size());
        telemetry.addData("# Incorrect Props", incorrectProps);
        telemetry.addLine("------------------------------");
        telemetry.addLine("SELECTED PROP");
        telemetry.addData("TYPE:", modelType);
        telemetry.addData("CONFIDENCE:", modelConfidence);
        telemetry.addData("HEIGHT:", modelHeight);
        telemetry.addData("X POS:", posX);

        if (!currentRecognitions.isEmpty() && posX != 0) {
            if (posX > 20 && posX < 350) {
                path = 1;
            } else if (posX > 350 && posX < 620) {
                path = 2;
            }
        } else {
            path = 0;
        }
        telemetry.addData("PATH:", path + "("+pathValues[path]+")");


        return path;
    }
*/



    /**
     * Detects AprilTags and stores them for later use.
     * @param tagID set to -1 to detect any tag and choose the first one found
     * @return whether it found the tag or not
     */
    public boolean detectAprilTag(int tagID) {
        boolean targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((tagID < 0) || (detection.id == tagID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;

                    break;  // don't look any further.
                }
            }
        }

        return targetFound;
    }

    @Deprecated
    public boolean isAprilTagOnTarget(double desiredDistance) {
        double rangeError = desiredTag.ftcPose.range-desiredDistance;
        double bearingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        // tolerances for April tag positioning can be modified here
        if (Math.abs(rangeError) < 1 && Math.abs(bearingError) < 2 && Math.abs(yawError) < 2) {
            return true; // on target and within tolerances
        } else {
            return false; // does not meet tolerance requirements
        }
    }

    /**
     * Checks if the robot is within tolerance of positioning relative to April Tag <br>
     * Used for autonomous
     * @param desiredDistance offset distance from 0
     * @param desiredBearing offset bearing from 0
     * @param desiredYaw offset yaw from 0
     * @return if robot is on target or should continue adjusting
     */
    public boolean isAprilTagOnTarget(double desiredDistance, double desiredBearing, double desiredYaw) {
        double rangeError = desiredTag.ftcPose.range-desiredDistance;
        double bearingError = desiredTag.ftcPose.bearing-desiredBearing;
        double yawError = desiredTag.ftcPose.yaw-desiredYaw;

        // tolerances for April tag positioning can be modified here
        if (Math.abs(rangeError) < 1 && Math.abs(bearingError) < 2 && Math.abs(yawError) < 2) {
            return true; // on target and within tolerances
        } else {
            return false; // does not meet tolerance requirements
        }
    }


    @Deprecated
    public void search(Direction direction, double power) {

        if (direction == Direction.LEFT) {
            // turn left
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);

        } else if (direction == Direction.RIGHT) {
            // turn right
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
        }

    }

    /**
     * Used in autonomous only, slowly turns robot in given direction to find April Tag
     */
    public void search(Direction direction) {
        double power = 0.2;

        if (direction == Direction.LEFT) {
            // turn left
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);

        } else if (direction == Direction.RIGHT) {
            // turn right
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
        }

    }

    /**
     * use to check if desiredTag is null.
     */
    public AprilTagDetection getDetectedAprilTag() {

        return desiredTag;
    }

    /**
     * @return if robot has an April Tag in its sight
     */
    public boolean hasAprilTag() {
        return desiredTag != null;
    }

    public void addAprilTagTelemetry() {
        telemetry.addData("April Tag ID", desiredTag.id);
        telemetry.addData("Distance", desiredTag.ftcPose.range);
        telemetry.addData("Bearing", desiredTag.ftcPose.bearing);
        telemetry.addData("Yaw", desiredTag.ftcPose.yaw);
    }


    /**
     * <strong>NOTE:</strong> <br>
     * this method only works if the camera is mounted on the front of the robot and parallel with the drivetrain
     * @param offsetDistance distance robot should stop in front of the AprilTag
     */
    @Deprecated
    public void driveToAprilTag(double offsetDistance) {
        // in case anything slips through
        if (desiredTag == null) return;

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double  rangeError      = (desiredTag.ftcPose.range - offsetDistance);
        double  headingError    = desiredTag.ftcPose.bearing;
        double  yawError        = desiredTag.ftcPose.yaw;
        double x;
        double yaw;
        double y;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        if (Math.abs(rangeError) < 2) {
            x  = Range.clip(rangeError * AUTO_GAINS.driving, -AUTO_SPEEDS.correctionDriving, AUTO_SPEEDS.correctionDriving); // drive
            yaw   = Range.clip(headingError * AUTO_GAINS.turning, -AUTO_SPEEDS.correctionTurning, AUTO_SPEEDS.correctionTurning) ; // turn
            y = Range.clip(-yawError * AUTO_GAINS.strafe, -AUTO_SPEEDS.correctionStrafe, AUTO_SPEEDS.correctionStrafe); // strafe
        } else {
            x  = Range.clip(rangeError * AUTO_GAINS.driving, -AUTO_SPEEDS.maxDriving, AUTO_SPEEDS.maxDriving); // drive
            yaw   = Range.clip(headingError * AUTO_GAINS.turning, -AUTO_SPEEDS.maxTurning, AUTO_SPEEDS.maxTurning) ; // turn
            y = Range.clip(-yawError * AUTO_GAINS.strafe, -AUTO_SPEEDS.maxStrafe, AUTO_SPEEDS.maxStrafe); // strafe
        }

        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }


    /**
     * Requires a tag from <strong>detectAprilTag()</strong> to run
     * @param offsetDistance changes how far away robot will be from the AprilTag
     * @param offsetBearing changes drivetrain relation of pointing directly at the AprilTag
     * @param offsetYaw changes robot's offset from the center of the AprilTag
     * @param cameraAngle angle camera is in relation to front of drivetrain
     */
    public void driveToAprilTag(double offsetDistance, double offsetBearing, double offsetYaw, double cameraAngle) {
        // in case anything slips through
        if (desiredTag == null) return;

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double  rangeError      = (desiredTag.ftcPose.range - offsetDistance);
        double  headingError    = (desiredTag.ftcPose.bearing - offsetBearing);
        double  yawError        = (desiredTag.ftcPose.yaw- offsetYaw);
        double x;
        double yaw;
        double y;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        if (Math.abs(rangeError) < 2) {
            x  = Range.clip(rangeError * AUTO_GAINS.driving, -AUTO_SPEEDS.correctionDriving, AUTO_SPEEDS.correctionDriving); // drive
            yaw   = Range.clip(headingError * AUTO_GAINS.turning, -AUTO_SPEEDS.correctionTurning, AUTO_SPEEDS.correctionTurning) ; // turn
            y = Range.clip(-yawError * AUTO_GAINS.strafe, -AUTO_SPEEDS.correctionStrafe, AUTO_SPEEDS.correctionStrafe); // strafe
        } else {
            x  = Range.clip(rangeError * AUTO_GAINS.driving, -AUTO_SPEEDS.maxDriving, AUTO_SPEEDS.maxDriving); // drive
            yaw   = Range.clip(headingError * AUTO_GAINS.turning, -AUTO_SPEEDS.maxTurning, AUTO_SPEEDS.maxTurning) ; // turn
            y = Range.clip(-yawError * AUTO_GAINS.strafe, -AUTO_SPEEDS.maxStrafe, AUTO_SPEEDS.maxStrafe); // strafe
        }

        double driveHeading = Math.toRadians(cameraAngle);

        double rotX = x * Math.cos(-driveHeading) - y * Math.sin(-driveHeading);
        double rotY = x * Math.sin(-driveHeading) + y * Math.cos(-driveHeading);

        // Calculate wheel powers.
        double leftFrontPower    =  rotX -rotY -yaw;
        double rightFrontPower   =  rotX +rotY +yaw;
        double leftBackPower     =  rotX +rotY -yaw;
        double rightBackPower    =  rotX -rotY +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }


    /**
     * Used for autonomous, stops drive motors when April Tag action is finished
     */
    public void stopDrivetrain() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }


    /**
     * For use in conjunction with setManualExposure()!
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

    }

    /*private void initTensorflow() {
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();
    }*/


    private void initVisionPortal() {
        // Create the vision portal by using a builder.
        if (usingAprilTag || usingTensorflow) {
            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, "eyeball"));

//            if (usingTensorflow) builder.addProcessor(tfod);
            if (usingAprilTag) builder.addProcessor(aprilTag);

            visionPortal = builder.build();
        }

    }


    /**
     * For use in conjunction with initAprilTag()! Should be surrounded with a try-catch.
     */
    public void setManualExposure(int exposureMS, int gain) throws InterruptedException, FailedInitializationException {
        //     Manually set the camera gain and exposure.
        //     This can only be called AFTER calling initAprilTag(), and only works for Webcams;

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal == null) {
            return;
        } else if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            throw new FailedInitializationException("Vision is not fully initialized; you failed to check for "+
                    "\"isCameraInitialized()\" before calling \"setManualExposure()\"");
        }


        // Set camera controls
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            Thread.sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        Thread.sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        Thread.sleep(20);

    }



    public boolean isCameraInitialized() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            initialized = false;
        } else {
            initialized = true;
        }
        return initialized;
    }


    /*-------------------------------------------------------------------------------------------
                                              ACTIONS
     -------------------------------------------------------------------------------------------*/


    /**
     * NOTE: THIS ACTION UTILIZES THE DRIVETRAIN! Do not use with parallel actions alongside RR trajectories!
     */
    @Deprecated
    public Action runAprilTagAction(int tagID, double offsetDistance, Direction direction) {
        return new Action() {
            private final int givenTagID = tagID;
            private final double givenOffsetDistance = offsetDistance;
            private final Direction givenDirection = direction;
            ElapsedTime timer;
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.startTime();
                    initialized = true;
                }



                if (!detectAprilTag(givenTagID)) {
                    search(givenDirection);

                } else {
                    driveToAprilTag(givenOffsetDistance);

                    // check if robot is where it is supposed to be and has been
                    // given enough time for slowdown and position correction;
                    // if so, give control of the drivetrain back to RR
                    // and end the action
                    boolean onTarget = isAprilTagOnTarget(givenOffsetDistance);
                    if (onTarget && timer.time(TimeUnit.SECONDS) > 2) {
                        stopDrivetrain();
                        return false;
                    } else if (!onTarget){
                        timer.reset();
                    }
                }

                return true;

            } // end run

        }; // end action class
    }


    /**
     * NOTE: THIS ACTION UTILIZES THE DRIVETRAIN! Do not use with parallel actions alongside RR trajectories!
     */
    public Action runAprilTagAction(
            int tagID,
            double offsetDistance, double offsetBearing, double offsetYaw, double cameraAngle,
            Direction direction) {

        return new Action() {
            private final int givenTagID = tagID;
            private final double givenDistance = offsetDistance;
            private final double givenBearing = offsetBearing;
            private final double givenYaw = offsetYaw;
            private final double givenCameraAngle = cameraAngle;
            private final Direction givenDirection = direction;
            private double startTime;
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    startTime = Actions.now();
                    initialized = true;
                }

                // get current runtime of action between checks of isAprilTagOnTarget(...)
                double t = Actions.now() - startTime;



                if (!detectAprilTag(givenTagID)) {
                    search(givenDirection);

                } else {
                    driveToAprilTag(
                            givenDistance,
                            givenBearing,
                            givenYaw,
                            givenCameraAngle
                    );

                    // check if robot is where it is supposed to be and has been
                    // given enough time for slowdown and position correction;
                    // if so, give control of the drivetrain back to RR
                    // and end the action
                    boolean onTarget = isAprilTagOnTarget(
                            givenDistance,
                            givenBearing,
                            givenYaw
                    );
                    if (onTarget && t > 2) {
                        stopDrivetrain();
                        return false;
                    } else if (!onTarget){
                        startTime = Actions.now(); // reset timer
                    }
                }

                return true;

            } // end run method

        }; // end action class
    } // end action creation method



}
