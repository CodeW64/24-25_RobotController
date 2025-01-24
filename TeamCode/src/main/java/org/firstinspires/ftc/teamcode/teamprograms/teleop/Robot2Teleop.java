package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotVision;
import org.firstinspires.ftc.teamcode.teamprograms.AscentStabilizer;
import org.firstinspires.ftc.teamcode.teamprograms.DistanceGetter;

/**
 * Welcome! <br>
 * Teleop Version: <strong>2.9.0 RELEASE</strong> <br>
 * STARTING POSITION/STATE: <strong>INTAKE_ACTIVE</strong> <br>
 * Please note: this program may be a crime scene of comments and technical debt.
 **/

 /*--------------------------
 * CONTROLS (most likely outdated)
 * --------------------------
 * Gamepad 1 (drivetrain)
 * [right_stick_y] - Right drive wheels
 * [left_stick_y] - Left drive wheels
 * [right_trigger] - Slide right
 * [left_trigger] - Slide left
 * [right_bumper] - Increase max speed
 * [left_bumper] - Lower max speed
 * [y_button] - run to an april tag (use for bucket tags)
 * [dpad_right + x_button] - reset pivot automatically if disconnected (DANGEROUS)
 * [dpad_up] - manually raise pivot in the mode
 * [dpad_down] - manually lower pivot in the mode
 * [back_button] - enable/disable duck spinner
 * --------------------------
 * Gamepad 2 (lift)
 * [right_stick_y] - extend/retract slides
 * [right_bumper] - divide samples (intake)
 * [right_bumper] - set intake wheels to empty (deposit)
 * [left_bumper] - set intake wheels to low speed (deposit)
 * [left_trigger] - flip between intake and deposit
 * [right_trigger] - move intake to and from deposit position (deposit)
 * [right_trigger] - attempt sample grab (intake)
 * [y_button] - empty intake when full (intake full)
 * [x_button/y_button] - override limit switch (retract and pivot states)
 * [dpad_up] - enter/exit HANG modes (from deposit mode)
 * [dpad_down + a_button] - enter MANUAL_OVERRIDE (DANGEROUS)
 * --------------------------
 * MANUAL_OVERRIDE (affects Gamepad 2 only)
 * NOTE: this mode is EXTREMELY DANGEROUS (yes, more than last year)
 *      BE ALERT: if the robot has disconnected, the robot does not know
 *                where the pivot position should be; during this,
 *                  DO NOT TRY TO EXIT TO AUTOMATIC, GO TO RESET
 *
 * Gamepad 2 (lift)
 * [right_stick_y] - extend/retract slides
 * [left_stick_y] - pivot slides
 * [right_bumper] - set intake wheels to high/low speed
 * [left_bumper] - fight gravity with the linear slide
 * [b_button] - reverse direction of intake wheels
 * [y_button] - deposit servo position
 * [left_trigger] - rest servo position
 * [right_trigger] - attempt sample grab
 * [x_button] - zero linear slide position
 * [dpad_down + a_button] - exit MANUAL_OVERRIDE
 */

@TeleOp(name = "Speedy Steeeve's Second Lap", group = "A")
@Config
public class Robot2Teleop extends LinearOpMode {


    // HARDWARE
    DcMotorEx frontRight, backRight, frontLeft, backLeft;
    DcMotorEx linearSlideRight, linearSlideLeft;
    DcMotorEx linearPivotRight, linearPivotLeft;
//    CRServo linearActuatorRight, linearActuatorLeft;
    CRServo intakeWheelR, intakeWheelL;
    Servo intakePivot;
    Servo specimenGrabber;
    CRServo duckSpinner;

//    DistanceSensor heightSensor;
    ColorRangeSensor sampleSensor;
    TouchSensor linearSlideSwitch;
    RobotVision glasses;
//    IMU imu;

    // SERVO POSITION VALUES (editable by FTC dashboard)
    public static class ServoValues {
        public double pivotIntakePos = 0.44; // 0.45
        public double pivotEjectSamplePos = 0.6;
        public double pivotDepositPos = 0.4;
        public double pivotRestPos = 0.52;
        public double pivotCarryPos = 0.7;
        public double specimenGrabberOpenPos = 0.58; // adjust
        public double specimenGrabberClosePos = 0.47; // adjust
    }
    public static ServoValues SERVO_VALUES = new ServoValues();

    // more servo variables
    final double INTAKE_POWER_MAX = 1.0;
    final double INTAKE_POWER_HOLD = 0.06;
    final double INTAKE_POWER_EMPTY = -0.3;
    final double INTAKE_POWER_ZERO = 0;

    // DUCK VALUES (editable by FTC dashboard)
    public static class DuckValues {
        public double spinStop = 0.0;
        public double spinRest = 0.15;
        public double spinActive = 0.3;
        public double spinHyperActive = 0.5;
    }
    public static DuckValues DUCK_VALUES = new DuckValues();


    // SENSOR VARIABLES (editable by FTC dashboard)
    public static class SensorVariables {
        public float sampleSensorGain = 1.0f;
        public double sampleDistance = 3.2;
        public double sampleCodeBlue = 0.007;
        public double glassesDistance = 11; // inches
        public double glassesBearing = 7; // degrees
        public double glassesYaw = 5; // degrees
        public double glassesCameraAngle = 60; // degrees
    }
    public static SensorVariables SENSOR_VARIABLES = new SensorVariables();

    // STABILIZER VARIABLES
    public static class StabilizerConstants {
        public double hookRadius = 1.25; // Inches
        public double drawBack = 1; // Inches
        public double distFromBarrier = 13.5; // X direction offset from barrier
        public double initialHookDist = 10; // Inches
        public double heightSensorOffsetX = 0; // Inches front from the pivot
        public double heightSensorOffsetY = -3.5; // Inches up from pivot
        public double highRungHeight = 35.5; // Inches
    }

    public static StabilizerConstants STABILIZER_CONSTANTS = new StabilizerConstants();

    /*public DistanceGetter heightGetter = (DistanceUnit unit) -> {
        // final double theta = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS); 
        // return 
        //     (heightSensor.getDistance(unit) - STABILIZER_CONSTANTS.heightSensorOffsetY) 
        //     * Math.cos(theta)
        //     + STABILIZER_CONSTANTS.heightSensorOffsetX * Math.sin(theta);
        return heightSensor.getDistance(DistanceUnit.INCH) - STABILIZER_CONSTANTS.heightSensorOffsetY;
    }; // cool interface, I will have to remember this technique!*/

    // SLIDE VARIABLES (editable by FTC dashboard)
    public static class SlideConstants {
        public double gravityCoefficient = 0.0005;
        public double extensionLimitIntake = 1780.0; // 312RPM-2500
        public double extensionLimitSpecimen = 2500.0; // 312RPM-3800 // FIXME: adjust to fit within limit
        public double extensionLimitWall = 500; // FIXME: adjust so pivot can lift up at max
        public double extensionLimitHang = 2700.0; // 312RPM-3800
        public double cushionRatio = 400.0;
        public double topBucketHeightAlternate = 3000.0; // 312RPM-4100 // 435RPM was 2930.0

    }
    public static SlideConstants SLIDE_CONSTANTS = new SlideConstants();

    // PIVOT VARIABLES (editable by FTC dashboard)
    public static class PivotConstants {
        public double cushionRatio = 400.0;
        public double kp = 0.005;
        public double ki = 0.0;
        public double kd = 0.0; // 0.0002
        public double gravityFeedForward = 0.002;
        public double retractSetSpeedMultiplier = 0.4;

        public double maxPos = 2000;
        public double minPos = 0;
        public double intakePos = 100;
        public double depositPos = 1650;
        public double depositRetractSetPos = 1800;
        public double hangOGPos = 2150;
        public double specimenGrabPos = 700; // adjust to right angle
        public double specimenPositionPos = 1200; // adjust to right angle
        public double specimenPlacePos = 1000; // adjust to right angle
        public double stabilizeReady = 1310;
    }
    public static PivotConstants PIVOT_CONSTANTS = new PivotConstants();

    final static double PIVOT_TICKS_PER_DEGREE = 2000.0 / 90.0; // (motor PPR / gear ratio) / 360
    final static double PIVOT_TICKS_PER_RAD = PIVOT_TICKS_PER_DEGREE * 180 / Math.PI;

    final double LIFT_TICKS_PER_INCH_EXTENDED = 384.5 / 537.7 * 3450.0 / 30.0;

    PIDController pivotController;

    /*public static class TimeConstants {
        public double handsInit = 13.0;
        public double handsDown = 9.0; // should be equal to handsDown
        public double handsUp = 9.5; // should be equal to handsUp
    }
    public static TimeConstants TIME_CONSTANTS = new TimeConstants();*/

    // ROBOT LIFT STATES
    enum LinearSlideStates {

        INTAKE_ACTIVE, INTAKE_ATTEMPT_SAMPLE, INTAKE_DIVIDE_SAMPLE, INTAKE_FULL, INTAKE_EMPTY,
        INTAKE_RETRACT, PIVOT_TO_DEPOSIT,

        DEPOSIT_ACTIVE,
        DEPOSIT_RETRACT_SET, DEPOSIT_RETRACT, PIVOT_TO_INTAKE,
        PIVOT_TO_DEPOSIT_REVERSE,

        PIVOT_TO_SPECIMEN_GRAB,
        SPECIMEN_GRAB, SPECIMEN_POSITION, SPECIMEN_PLACE,
        SPECIMEN_RETRACT,

        PIVOT_TO_HANG_TIME_OG, HANG_TIME_OG,

        MANUAL_OVERRIDE,

        PIVOT_MANUAL_RESET,


//        PIVOT_TO_STABILIZE_ROBOT,
//        STABILIZE_ROBOT,
//        HANG_TIME_AUTOMATIC_HANDS,
//        HANG_TIME_AUTOMATIC_ARM, HANG_TIME_AUTOMATIC_ARM_ALTERNATE,
//        HANG_TIME_MANUAL
    }
    LinearSlideStates linearSlideState;

    enum ExtensionLimits {
        INTAKE, DEPOSIT, SPECIMEN, WALL, HANG
    }

    /*enum ActuatorHangStates {
        HANDS_INITIALIZE,
        HANDS_UP, HANDS_DOWN,
        HANDS_AT_REST,
        HANDS_RESET
    }
    ActuatorHangStates actuatorHangState;
    double storedActuatorTime = 0.0;
    boolean isRobot2ndLevelAscending = false;*/


    ElapsedTime lightTimer, actuatorTimer, pidTimer;

//    AscentStabilizer ascentStabilizer = new AscentStabilizer(heightGetter);


    // constant variables
    final double SLIDE_SPEED = 1.0;
    final double PIVOT_SPEED = 1.0;
//    final double ACTUATOR_SPEED = 1.0;

    boolean tankDrive = true;
    boolean disableDuck = false;
    boolean runningToBucketAprilTag = false;
    boolean overridePID = false;
    boolean camera = true; // disable if camera not in use or if it doesn't exist
    boolean isRunningPivotToPosition = false;
    boolean specimanning = false;



    @Override
    public void runOpMode() {
        // "Method 'runOpMode' is too complex to analyze by data flow algorithm" -AS

        initHardware();

        pivotController = new PIDController(PIVOT_CONSTANTS.kp, PIVOT_CONSTANTS.ki, PIVOT_CONSTANTS.kd);
        pivotController.setTolerance(10);
        double pivotPIDSpeedMultiplier = 1.0;

        // VARIABLES/OBJECTS (miscellaneous) ----------------------------------------------------------

        // drivetrain
        int driveSpeedIndex = 1; // change this to the index of the speed you want to start at
        double[] driveSpeedRange = {0.5, 0.75, 1};
        double driveSpeedFactor = 1;

        // tank drive
        double rightPower = 0;
        double leftPower = 0;
        double slideToRight = 0;
        double slideToLeft = 0;

        // mecanum drive
        double powerX = 0;
        double powerY = 0;
        double powerRX = 0;


        // BUTTON CHECKS

        boolean checkGTwoRB = true;
        boolean checkGTwoLB = true;

        boolean checkGTwoRT = true;
        boolean checkGTwoLT = true;

        boolean checkGTwoDDOWN = true;
        boolean checkGTwoA = true;

        boolean checkGOneRB = true;
        boolean checkGOneLB = true;

        boolean checkGTwoX = true;
        boolean checkGTwoY = true;

        boolean checkGTwoDUP = true;
        boolean checkGTwoDLEFT = true;

        boolean checkGTwoDRIGHT = true;

        boolean checkGOneDDOWN = true;
        boolean checkGOneDUP = true;

        boolean checkGOneDRIGHT = true;
        boolean checkGOneX = true;
        boolean checkGOneY = true;

        boolean checkGOneB = true;
        boolean checkGOneA = true;

        boolean checkGOneBACK = true;

        lightTimer = new ElapsedTime();
        actuatorTimer = new ElapsedTime();
        pidTimer = new ElapsedTime();

        // STATE MACHINE LOGIC

        boolean isStateInitialized = false;
        boolean isIntakeProtected = false;
        boolean isGrabberOpen = true;
        boolean isArmPositionSet = true;
//        boolean isGoingToHangTime = false; // True when going to hang from deposit or intake; false otherwise
        boolean isExitingHangTime = false; // True when aborting from STABILIZE_ROBOT before PIVOT_TO_INTAKE; false otherwise

        // ACTUATOR LOGIC
        boolean isActuatorStateInitialized = false;

        // WAIT LOOP ----------------------------------------------------------------------------

        lightTimer.startTime();
        actuatorTimer.startTime();
        pidTimer.startTime();


        while (opModeInInit()) {

            // change drive mode
            if (gamepad1.right_bumper) {
                tankDrive = true;
            } else if (gamepad1.left_bumper) {
                tankDrive = false;
            }


            // START
            telemetry.addLine("TELEOP VERSION 2.9.0 RELEASE");
            telemetry.addLine("-------------------------");
            telemetry.addData("TANK DRIVE", tankDrive);
            telemetry.addLine("CONTROLLER 1  RIGHT BUMPER: TANK DRIVE");
            telemetry.addLine("CONTROLLER 1 LEFT BUMPER: MECANUM DRIVE");
            telemetry.addLine("-------------------------");
            telemetry.addLine("Press Start");
            telemetry.update();
        }

        waitForStart();

        lightTimer.reset();
        actuatorTimer.reset();
        pidTimer.reset();
        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
//        actuatorHangState = ActuatorHangStates.HANDS_INITIALIZE;
        int linearPivotTargetPosition = (int)PIVOT_CONSTANTS.intakePos;

//        specimenGrabber.setPosition(SERVO_VALUES.specimenGrabberOpenPos);

        double desiredArmTheta = 0; 
        double desiredArmLength = 0; 

        // RUN LOOP -----------------------------------------------------------------------------

        while (opModeIsActive()) {
//            ascentStabilizer.h = STABILIZER_CONSTANTS.highRungHeight;
//            ascentStabilizer.x = STABILIZER_CONSTANTS.distFromBarrier;
//            ascentStabilizer.r = STABILIZER_CONSTANTS.hookRadius;


//            linearActuatorRight.setPower(0);
//            linearActuatorLeft.setPower(0);

            // BUTTON CHECKS
            if (!gamepad2.right_bumper) checkGTwoRB = false;
            if (!gamepad2.left_bumper) checkGTwoLB = false;

            if (gamepad2.right_trigger < 0.07)  checkGTwoRT = false;
            if (gamepad2.left_trigger < 0.07)  checkGTwoLT = false;

            if (!gamepad2.dpad_down) checkGTwoDDOWN = false;
            if (!gamepad2.a) checkGTwoA = false;

            if (!gamepad1.right_bumper) checkGOneRB = false;
            if (!gamepad1.left_bumper) checkGOneLB = false;

            if (!gamepad2.x) checkGTwoX = false;
            if (!gamepad2.y) checkGTwoY = false;

            if (!gamepad2.dpad_up) checkGTwoDUP = false;
            if (!gamepad2.dpad_left) checkGTwoDLEFT = false;

            if (!gamepad2.dpad_right) checkGTwoDRIGHT = false;

            if (!gamepad1.dpad_down) checkGOneDDOWN = false;
            if (!gamepad1.dpad_up) checkGOneDUP = false;

            if (!gamepad1.dpad_right) checkGOneDRIGHT = false;
            if (!gamepad1.x) checkGOneX = false;
            if (!gamepad1.y) checkGOneY = false;
            if (!gamepad1.b) checkGOneB = false;
            if (!gamepad1.a) checkGOneA = false;

            if (!gamepad1.back) checkGOneBACK = false;


// DUCK ------------------------------------------------------------------------------------------

            if (gamepad1.back && !checkGOneBACK && !disableDuck) {
                checkGOneBACK = true;
                duckSpinner.setPower(DUCK_VALUES.spinStop);
                disableDuck = true;
            } else if (gamepad1.back && !checkGOneBACK && disableDuck) {
                checkGOneBACK = true;
                disableDuck = false;
            }


// LIFT -----------------------------------------------------------------------------------------

            // gather values
            double linearSlidePower = 0.0;
            double linearSlideAvgPosition = (linearSlideRight.getCurrentPosition() + linearSlideLeft.getCurrentPosition()) / 2.0;
            boolean limitSwitch = linearSlideSwitch.isPressed();

            double linearPivotAvgPosition = (linearPivotRight.getCurrentPosition() + linearPivotLeft.getCurrentPosition()) / 2.0;

            double currentSampleDistance = sampleSensor.getDistance(DistanceUnit.CM);



            // enter MANUAL_OVERRIDE
            if (gamepad2.dpad_down && !checkGTwoDDOWN && gamepad2.a && !checkGTwoA &&
                linearSlideState != LinearSlideStates.MANUAL_OVERRIDE) {
                checkGTwoDDOWN = true;
                checkGTwoA = true;
                isStateInitialized = false;
                linearSlideState = LinearSlideStates.MANUAL_OVERRIDE;
            }


            // enter PIVOT_MANUAL_RESET
            if (gamepad1.dpad_right && !checkGOneDRIGHT && gamepad1.x && !checkGOneX &&
                linearSlideState != LinearSlideStates.PIVOT_MANUAL_RESET) {
                checkGOneDRIGHT = true;
                checkGOneX = true;
                isStateInitialized = false;
                linearSlideState = LinearSlideStates.PIVOT_MANUAL_RESET;
            }

// LIFT STATE MACHINE ----------------------------------------------------------------------------

            switch (linearSlideState) {

            // lift in position to grab samples
            // speed of intake wheels is at rest
                case INTAKE_ACTIVE:

                    if (!isStateInitialized) {
                        if (!disableDuck) duckSpinner.setPower(DUCK_VALUES.spinRest);

                        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);


                        isRunningPivotToPosition = false;
                        linearPivotRight.setPower(0);
                        linearPivotLeft.setPower(0);

                        lightTimer.reset();
                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }

                    // SLIDES

                    linearSlidePower = calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.INTAKE);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);

                    // ATTEMPT/DIVIDE SAMPLE

                    // attempt to grab a sample (if safe)
                    // slide exit 312RPM-500
                    if (gamepad2.right_trigger > 0.1 && !checkGTwoRT &&
                        linearSlideAvgPosition > 350) {
                        checkGTwoRT = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ATTEMPT_SAMPLE;
                    } else if (gamepad2.right_bumper && !checkGTwoRB &&
                        linearSlideAvgPosition > 350) {
                        checkGTwoRB = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_DIVIDE_SAMPLE;
                    }

                    // EXIT INTAKE

                    // start sequence to pivot to deposit
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        specimanning = false;
//                        isGoingToHangTime = false;

                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);

                        // FIXME: may break pivots, comment out if so
                        linearPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearPivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        linearPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        linearPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_RETRACT;
                    }

                    // start sequence to pivot to specimen
                    /*if (gamepad2.left_bumper && !checkGTwoLB) {
                        checkGTwoLB = true;
                        specimanning = true;
                        isGoingToHangTime = false;

                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);

                        linearPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        linearPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_RETRACT;
                    }*/

                    // begin pivot to hang (3rd level)
                    /* if (gamepad2.dpad_up && !checkGTwoDUP) {
                         checkGTwoDUP = true;
                         specimanning = false;
                         isGoingToHangTime = true;

                         linearSlideRight.setPower(0);
                         linearSlideLeft.setPower(0);

                         linearPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                         linearPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                         isStateInitialized = false;
                         linearSlideState = LinearSlideStates.INTAKE_RETRACT;
                     }*/
                    break;

            // robot attempts to grab a sample
            // intake cannot be messed with while in this mode
                case INTAKE_ATTEMPT_SAMPLE:

                    if (!isStateInitialized) {
                        intakeWheelR.setPower(INTAKE_POWER_MAX);
                        intakeWheelL.setPower(INTAKE_POWER_MAX);
                        intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);

                        lightTimer.reset();
                        isStateInitialized = true;
                    }


                    // SLIDES
                    linearSlidePower = calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.INTAKE);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);

                    // SERVOS
                    // finish claw machine grab after a set amount of time
                    if (lightTimer.seconds() > 0.9) {

                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;

                        // figure out if robot has grabbed sample
                        if (isPossessingSample(currentSampleDistance)) {
                            // robot has successfully acquired a sample
                            linearSlideState = LinearSlideStates.INTAKE_FULL;
                        } else {
                            // robot did not get sample
                            linearSlideState = LinearSlideStates.INTAKE_EMPTY;
                        }
                    } else if (lightTimer.seconds() > 0.6) {
                        // bring intake up for a short period to secure sample
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                    }

                    // ABORT ATTEMPT

                    // leave sample attempt early if mistaken
                    if (gamepad2.right_trigger > 0.1 && !checkGTwoRT) {
                        checkGTwoRT = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_EMPTY;
                    }
                    break;

            // attempt to split the samples close together for easier grabbing
                case INTAKE_DIVIDE_SAMPLE:

                    if (!isStateInitialized) {
                        intakeWheelR.setPower(INTAKE_POWER_EMPTY);
                        intakeWheelL.setPower(INTAKE_POWER_EMPTY);
                        intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);

                        isArmPositionSet = false;
                        lightTimer.reset();
                        isStateInitialized = true;
                    }


                    // SLIDES
                    linearSlidePower = calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.INTAKE);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);

                    // SERVOS
                    // finish claw machine grab after a set amount of time
                    if (isArmPositionSet && lightTimer.seconds() > 1) {
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
                    } else if (isArmPositionSet && lightTimer.seconds() > 0.8) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                    }

                    // ABORT DIVIDE

                    // leave sample divide early if mistaken
                    if (gamepad2.right_bumper && !checkGTwoRB) {
                        checkGTwoRB = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
                    }
                    break;

            // robot is in possession of a sample
            // speed of intake wheels changes to accommodate
                case INTAKE_FULL:

                    if (!isStateInitialized) {
                        if (!disableDuck) duckSpinner.setPower(DUCK_VALUES.spinActive);

                        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
                        isIntakeProtected = true;
                        intakeWheelR.setPower(INTAKE_POWER_HOLD);
                        intakeWheelL.setPower(INTAKE_POWER_HOLD);

                        isStateInitialized = true;
                    }


                    // SLIDES
                    linearSlidePower = calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.INTAKE);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);


                    // for ejecting wrong colored samples
                    if (gamepad2.y) {
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        isIntakeProtected = false;
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        linearSlideState = LinearSlideStates.INTAKE_EMPTY;
                    }



                    // EXIT INTAKE

                    // start sequence to pivot to deposit
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
//                        isGoingToHangTime = false;

                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);

                        // FIXME: may break pivots, comment out if so
                        linearPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearPivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        linearPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        linearPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                        isStateInitialized = false;
                        isIntakeProtected = false;
                        linearSlideState = LinearSlideStates.INTAKE_RETRACT;
                    }

                    // begin pivot to hang
                    // if (gamepad2.dpad_up && !checkGTwoDUP) {
                    //     checkGTwoDUP = true;
                    //     isGoingToHangTime = true;

                    //     linearSlideRight.setPower(0);
                    //     linearSlideLeft.setPower(0);

                    //     linearPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    //     linearPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                    //     isStateInitialized = false;
                    //     isIntakeProtected = false;
                    //     linearSlideState = LinearSlideStates.INTAKE_RETRACT;
                    // }
                    break;

            // spit out sample collected (if it did)
                case INTAKE_EMPTY:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotEjectSamplePos);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);
                        lightTimer.reset();
                        isStateInitialized = true;
                    }

                    linearSlidePower = calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.INTAKE);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);

                    if (lightTimer.seconds() > 0.5) {
                        // exit emptying
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
                    } else if (lightTimer.seconds() > 0.1) {
                        // give some time for intake pivot to lift before emptying
                        intakeWheelR.setPower(INTAKE_POWER_EMPTY);
                        intakeWheelL.setPower(INTAKE_POWER_EMPTY);
                    }

                    break;

            // retract slide so it can pivot
                case INTAKE_RETRACT:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
                        lightTimer.reset();
                        isStateInitialized = true;
                    }

                    if (!limitSwitch) {
                        double timeAccel = Math.min((lightTimer.seconds()*2), SLIDE_SPEED);
                        linearSlideRight.setPower(-timeAccel);
                        linearSlideLeft.setPower(-timeAccel);
                    } else {
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                    }

                    // EXIT

                    // start exiting right before slide hits 0
                    // (attempts to make transition faster and smoother)
                    // slide exit 312RPM-800
                    if (linearSlideAvgPosition < 570 || isLinearSlideFullyRetracted(limitSwitch)) {
                        isStateInitialized = false;

                        if (specimanning) {
                            linearSlideState = LinearSlideStates.PIVOT_TO_SPECIMEN_GRAB;
                        } else {
                            // normal mode
                            linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT;
                        }

                        /*if (specimanning) {
                            linearSlideState = LinearSlideStates.PIVOT_TO_SPECIMEN_GRAB;
                        } else if (isGoingToHangTime) {
                            linearSlideState = LinearSlideStates.PIVOT_TO_STABILIZE_ROBOT;
                        } else {
                            // normal mode
                            linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT;
                        }*/

                    }


                    // ABORT

                    // go back to intake if mistaken
                    // for deposit/intake
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
                    }

                    // go back to intake if mistaken
                    // for specimen
                    if (gamepad2.left_bumper && !checkGTwoLB) {
                        checkGTwoLB = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
                    }

                    // go back to intake if mistaken
                    // for hang
                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
                    }

                    break;

            // move lift from intake mode to deposit mode
                case PIVOT_TO_DEPOSIT:

                    if (!isStateInitialized) {
                        linearPivotTargetPosition = (int)PIVOT_CONSTANTS.depositPos;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();

                        lightTimer.reset();
                        isIntakeProtected = true;
                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }


                    // stop slides once finished retracting
                    // (slides started retracting in INTAKE_RETRACT)
                    if (isLinearSlideFullyRetracted(limitSwitch)) {
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    } else if (!isArmPositionSet) {
                        linearSlideRight.setPower(-SLIDE_SPEED);
                        linearSlideLeft.setPower(-SLIDE_SPEED);
                    }


                    // EXIT

                    // make deposit accessible once lift has finished pivoting
                    // (and once slide has finished retracting)
                    // NOTE: pivot finishes in deposit mode
                    if (Math.abs(linearPivotAvgPosition - PIVOT_CONSTANTS.depositPos) < 750 &&
                        isArmPositionSet) {
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_ACTIVE;
                    }

                    // ABORT

                    // go back to intake if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                    }
                    break;


            // lift in position to score samples more accurately and safely (ALTERNATE)
            // NOTE: drivetrain must turn around in this mode to deposit
                case DEPOSIT_ACTIVE:

                    if (!isStateInitialized) {
                        if (!disableDuck) duckSpinner.setPower(DUCK_VALUES.spinHyperActive);

                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isIntakeProtected = true;
                        intakeWheelR.setPower(INTAKE_POWER_HOLD);
                        intakeWheelL.setPower(INTAKE_POWER_HOLD);

                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }

                    // SLIDES
                    linearSlidePower = calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.DEPOSIT);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);


                    // SERVOS

                    // move intake pivot to and from deposit mode
                    if (gamepad2.right_trigger > 0.1 && !checkGTwoRT && isIntakeProtected) {
                        checkGTwoRT = true;
                        intakePivot.setPosition(SERVO_VALUES.pivotDepositPos);
                        isIntakeProtected = false;
                    } else if (gamepad2.right_trigger > 0.1 && !checkGTwoRT && !isIntakeProtected) {
                        checkGTwoRT = true;
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isIntakeProtected = true;
                    }

                    // manipulate held sample by holding it or depositing
                    if (gamepad2.left_bumper) {
                        intakeWheelR.setPower(INTAKE_POWER_HOLD);
                        intakeWheelL.setPower(INTAKE_POWER_HOLD);
                    } else if (gamepad2.right_bumper) {
                        intakeWheelR.setPower(INTAKE_POWER_EMPTY);
                        intakeWheelL.setPower(INTAKE_POWER_EMPTY);
                    }

                    // EXIT

                    // begin leave of deposit mode
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
//                        isGoingToHangTime = false;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_RETRACT_SET;
                    }

                    // for hang OG
                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_HANG_TIME_OG;
                    }

                    // begin pivot to hang (3rd level)
                    /* if (gamepad2.dpad_up && !checkGTwoDUP) {
                         checkGTwoDUP = true;
                         isGoingToHangTime = true;
                         isStateInitialized = false;
                         linearSlideState = LinearSlideStates.DEPOSIT_RETRACT_SET;
                     }*/

                    break;

            // ready lift to retract safely (ALTERNATE)
                case DEPOSIT_RETRACT_SET:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        intakeWheelR.setPower(INTAKE_POWER_HOLD);
                        intakeWheelL.setPower(INTAKE_POWER_HOLD);

                        linearPivotTargetPosition = (int)PIVOT_CONSTANTS.depositRetractSetPos;
                        pivotPIDSpeedMultiplier = PIVOT_CONSTANTS.retractSetSpeedMultiplier;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();

                        lightTimer.reset();
                        isStateInitialized = true;
                    }



                    // EXIT

                    // retract slides when pivot has been set
                    if (Math.abs(linearPivotAvgPosition - PIVOT_CONSTANTS.depositRetractSetPos) < 20) {
                        pivotPIDSpeedMultiplier = 1.0;
                        isStateInitialized = false;

                        // Going to hang time if so told to
                        linearSlideState = LinearSlideStates.DEPOSIT_RETRACT;
                    }

                    // ABORT

                    // go back to deposit if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        pivotPIDSpeedMultiplier = 1.0;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT_REVERSE;
                    }

                    // for hang
                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        pivotPIDSpeedMultiplier = 1.0;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT_REVERSE;
                    }
                    break;

            // retract slide so it can pivot
                case DEPOSIT_RETRACT:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        lightTimer.reset();
                        isStateInitialized = true;
                    }

                    if (!limitSwitch) {
                        double timeAccel = Math.min((lightTimer.seconds()*2), SLIDE_SPEED);
                        linearSlideRight.setPower((-timeAccel)+SLIDE_CONSTANTS.gravityCoefficient);
                        linearSlideLeft.setPower((-timeAccel)+SLIDE_CONSTANTS.gravityCoefficient);
                    } else {
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                    }


                    // exit mode if slide has gone far enough to safely start pivoting
                    // slide exit 312RPM-1800
                    if (linearSlideAvgPosition < 1280 || isLinearSlideFullyRetracted(limitSwitch)) {
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;

                        /*if (!isGoingToHangTime) {
                            linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                        } else {
                            linearSlideState = LinearSlideStates.PIVOT_TO_STABILIZE_ROBOT;
                        }*/
                    }

                    // ABORT

                    // go back to deposit if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        isIntakeProtected = true;
                        // alternate deposit pivots slightly upon retract set
                        // must go back to pivoting to deposit to fully reset
                        linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT_REVERSE;
                    }

                    // for hang
                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        isIntakeProtected = true;
                        // alternate deposit pivots slightly upon retract set
                        // must go back to pivoting to deposit to fully reset
                        linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT_REVERSE;
                    }
                    break;

            // move lift from deposit mode to intake mode
                case PIVOT_TO_INTAKE:

                    if (!isStateInitialized) {
                        linearPivotTargetPosition = (int)PIVOT_CONSTANTS.intakePos;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();


                        lightTimer.reset();
                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }


                    // stop slides once finished retracting
                    // (slides started retracting in DEPOSIT_RETRACT)
                    if (isLinearSlideFullyRetracted(limitSwitch)) {
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    } else if (!isArmPositionSet){
                        linearSlideRight.setPower(-SLIDE_SPEED);
                        linearSlideLeft.setPower(-SLIDE_SPEED);
                    }

                    // EXIT

                    // make intake accessible once lift has finished pivoting enough
                    // (and once slide has finished retracting)
                    if (Math.abs(linearPivotAvgPosition - PIVOT_CONSTANTS.intakePos) < 300 &&
                        isArmPositionSet) {
                        // brace for impact!
                        linearPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        linearPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        isRunningPivotToPosition = false;
                        linearPivotRight.setPower(0);
                        linearPivotLeft.setPower(0);

                        isStateInitialized = false;

                        // for in case deposit pivot was canceled
                        if (isPossessingSample(currentSampleDistance)) {
                            linearSlideState = LinearSlideStates.INTAKE_FULL;
                        } else {
                            linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
                        }
                    }

                    // ABORT

                    // go back to deposit if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        isStateInitialized = false;
                        isIntakeProtected = true;

                        linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT;
                    }
                    break;


            // cancel pivot to intake while pivot is shifted and ready to retract
                case PIVOT_TO_DEPOSIT_REVERSE:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);

                        linearPivotTargetPosition = (int)PIVOT_CONSTANTS.depositPos;
                        pivotPIDSpeedMultiplier = PIVOT_CONSTANTS.retractSetSpeedMultiplier;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();

                        lightTimer.reset();
                        isStateInitialized = true;
                    }



                    // EXIT

                    // go back to deposit
                    // NOTE: finishes pivoting there
                    if (Math.abs(linearPivotAvgPosition - PIVOT_CONSTANTS.depositPos) < 100) {
                        pivotPIDSpeedMultiplier = 1.0;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_ACTIVE;
                    }

                    // ABORT

                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        pivotPIDSpeedMultiplier = 1.0;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_RETRACT_SET;
                    }

                    // for hang OG
                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        pivotPIDSpeedMultiplier = 1.0;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_HANG_TIME_OG;
                    }
                    break;


            // move from intake to specimen mode, and return from specimen place
                case PIVOT_TO_SPECIMEN_GRAB:
                    if (!isStateInitialized) {
                        linearPivotTargetPosition = (int)PIVOT_CONSTANTS.specimenGrabPos;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();

                        lightTimer.reset();
                        isIntakeProtected = true;
                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }


                    // stop slides once finished retracting
                    // (slides started retracting in INTAKE_RETRACT)
                    if (isLinearSlideFullyRetracted(limitSwitch)) {
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    } else if (!isArmPositionSet) {
                        linearSlideRight.setPower(-SLIDE_SPEED);
                        linearSlideLeft.setPower(-SLIDE_SPEED);
                    }


                    // EXIT

                    // make specimen grab accessible once lift has finished pivoting
                    // (and once slide has finished retracting)
                    if (Math.abs(linearPivotAvgPosition - PIVOT_CONSTANTS.specimenGrabPos) < 20 &&
                            isArmPositionSet) {
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.SPECIMEN_GRAB;
                    }

                    // ABORT

                    // go back to intake if mistaken
                    if (gamepad2.left_bumper && !checkGTwoLB) {
                        checkGTwoLB = true;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                    }
                    break;


            // allows robot to grab specimens from the observation zone wall
                case SPECIMEN_GRAB:
                    if (!isStateInitialized) {
                        // HOLD ON!!!
                        if (!disableDuck) duckSpinner.setPower(DUCK_VALUES.spinStop);

                        linearPivotTargetPosition = (int)PIVOT_CONSTANTS.specimenGrabPos;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();

                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        intakeWheelR.setPower(0);
                        intakeWheelL.setPower(0);

                        specimenGrabber.setPosition(SERVO_VALUES.specimenGrabberClosePos);
                        isGrabberOpen = false;
                        isStateInitialized = true;
                    }

                    // SLIDES
                    linearSlidePower = calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.WALL);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);


                    // SERVOS

                    // move specimen grabber from open and close positions
                    if (gamepad2.right_bumper && !checkGTwoRB) {
                        checkGTwoRB = true;
                        if (isGrabberOpen) {
                            // close grabber
                            specimenGrabber.setPosition(SERVO_VALUES.specimenGrabberClosePos);
                            isGrabberOpen = false;
                        } else {
                            // open grabber
                            specimenGrabber.setPosition(SERVO_VALUES.specimenGrabberOpenPos);
                            isGrabberOpen = true;
                        }
                    }


                    // EXIT

                    // pivot to positioning specimen hang once successful grab off wall
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        specimenGrabber.setPosition(SERVO_VALUES.specimenGrabberClosePos);
                        isGrabberOpen = false;
                        isStateInitialized = false;
//                        linearSlideState = LinearSlideStates.SPECIMEN_POSITION;
                        linearSlideState = LinearSlideStates.SPECIMEN_POSITION;
                    }

                    // go back to intake mode
                    if (gamepad2.left_bumper && !checkGTwoLB) {
                        checkGTwoLB = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                    }
                    break;


            // allows robot to position for a specimen hang
                case SPECIMEN_POSITION:
                    if (!isStateInitialized) {
                        if (!disableDuck) duckSpinner.setPower(DUCK_VALUES.spinRest);

                        linearPivotTargetPosition = (int)PIVOT_CONSTANTS.specimenPositionPos;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();

                        intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);

                        isStateInitialized = true;
                    }

                    // SLIDES

                    linearSlidePower = calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.SPECIMEN);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);


                    // EXIT

                    // go to specimen place when positioned correctly for a hang
                    if (gamepad2.right_trigger > 0.1 && !checkGTwoRT) {
                        checkGTwoRT = true;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.SPECIMEN_PLACE;
                    }


                    // ABORT

                    // go back to specimen grab if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.SPECIMEN_RETRACT;
                    }

                    break;


            // allows robot to hang the specimen on the bar
                case SPECIMEN_PLACE:
                    if (!isStateInitialized) {
                        if (!disableDuck) duckSpinner.setPower(DUCK_VALUES.spinHyperActive);

                        linearPivotTargetPosition = (int)PIVOT_CONSTANTS.specimenPlacePos;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();

                        intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);

                        isStateInitialized = true;
                    }

                    // SERVOS

                    // move specimen grabber from open and close positions
                    if (gamepad2.right_bumper && !checkGTwoRB) {
                        checkGTwoRB = true;
                        if (isGrabberOpen) {
                            // close grabber
                            specimenGrabber.setPosition(SERVO_VALUES.specimenGrabberClosePos);
                            isGrabberOpen = false;
                        } else {
                            // open grabber
                            specimenGrabber.setPosition(SERVO_VALUES.specimenGrabberOpenPos);
                            isGrabberOpen = true;
                        }
                    }


                    // SLIDES

                    linearSlidePower = calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.SPECIMEN);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);


                    // EXIT

                    // begin shift to grab upon a successful placement
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.SPECIMEN_RETRACT;
                    }

                    // flip back to specimen position if mistaken
                    if (gamepad2.right_trigger > 0.1 && !checkGTwoRT) {
                        checkGTwoRT = true;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.SPECIMEN_POSITION;
                    }
                    break;


            // go back from placing to grabbing specimens
                case SPECIMEN_RETRACT:
                    if (!isStateInitialized) {
                        if (!disableDuck) duckSpinner.setPower(DUCK_VALUES.spinRest);

                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);

                        lightTimer.reset();
                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }

                    // SLIDES

                    // stop slides once finished retracting
                    if (isLinearSlideFullyRetracted(limitSwitch)) {
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    } else if (!isArmPositionSet) {
                        double timeAccel = Math.min((lightTimer.seconds()*2), SLIDE_SPEED);
                        linearSlideRight.setPower(-timeAccel);
                        linearSlideLeft.setPower(-timeAccel);
                    }


                    // EXIT

                    // go to specimen grab mode once slides have finished retracting
                    // maybe eventually finish the retract in specimen grab to save time
                    if (isArmPositionSet) {
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.SPECIMEN_GRAB;
                    }


                    // ABORT
                    // nothing currently
                    break;

                case PIVOT_TO_HANG_TIME_OG:
                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);

                        linearPivotTargetPosition = (int)PIVOT_CONSTANTS.hangOGPos;
                        pivotPIDSpeedMultiplier = PIVOT_CONSTANTS.retractSetSpeedMultiplier;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();

                        lightTimer.reset();
                        isIntakeProtected = true;
                        isStateInitialized = true;
                    }



                    // EXIT

                    // make deposit accessible once lift has finished pivoting
                    // (and once slide has finished retracting)
                    // NOTE: pivot finishes in deposit mode
                    if (Math.abs(linearPivotAvgPosition - PIVOT_CONSTANTS.hangOGPos) < 300) {
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_ACTIVE;
                    }

                    // ABORT

                    // go back to deposit if mistaken
                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT_REVERSE;
                    }
                    break;

                case HANG_TIME_OG:
                    if (!isStateInitialized) {
                        if (!disableDuck) duckSpinner.setPower(DUCK_VALUES.spinStop);

                        intakeWheelR.setPower(0);
                        intakeWheelL.setPower(0);
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isStateInitialized = true;
                    }

                    // SLIDES
                    linearSlidePower = calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.HANG);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);

                    // TODO: eventually add a jitter button for pivot?

                    // EXIT

                    // go back to deposit
                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT_REVERSE;
                    }
                    break;

                case MANUAL_OVERRIDE: {

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        duckSpinner.setPower(DUCK_VALUES.spinStop);
                        isRunningPivotToPosition = false;
                        overridePID = true;
                        linearPivotRight.setPower(0);
                        linearPivotLeft.setPower(0);
                        isStateInitialized = true;
                    }

                    linearSlidePower = 0;

                    // fight gravity
                    if (gamepad2.dpad_up) {
                        linearSlidePower += SLIDE_CONSTANTS.gravityCoefficient;
                    }

                    // set power to lift motors
                    linearSlidePower += calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.INTAKE);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);

                    double linearPivotPower = calculateManualPivotPower(linearPivotAvgPosition);
                    linearPivotRight.setPower(linearPivotPower);
                    linearPivotLeft.setPower(linearPivotPower);


                    // change speed of intake wheels
                    if (gamepad2.left_bumper) {
                        // wheel power low
                        intakeWheelR.setPower(INTAKE_POWER_HOLD);
                        intakeWheelL.setPower(INTAKE_POWER_HOLD);
                    } else if (gamepad2.right_bumper) {
                        // wheel power active
                        intakeWheelR.setPower(INTAKE_POWER_MAX);
                        intakeWheelL.setPower(INTAKE_POWER_MAX);
                    } else if (gamepad2.b) {
                        // wheel power deposit
                        intakeWheelR.setPower(INTAKE_POWER_EMPTY);
                        intakeWheelL.setPower(INTAKE_POWER_EMPTY);
                    }

                    // move the intake pivot (servo)
                    if (gamepad2.left_trigger > 0.1) {
                        // pivot to default position
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                    } else if (gamepad2.right_trigger > 0.1) {
                        // pivot to grab position
                        intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);
                    } else if (gamepad2.y) {
                        // pivot protected
                        intakePivot.setPosition(SERVO_VALUES.pivotEjectSamplePos);
                    }

                    // reset the position of the linear slide
                    if (gamepad2.x) {
                        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }


                    // move hands for hanging
                    /*if (gamepad2.dpad_up) {
                        linearActuatorRight.setPower(ACTUATOR_SPEED);
                        linearActuatorLeft.setPower(ACTUATOR_SPEED);
                    } else if (gamepad2.dpad_down) {
                        linearActuatorRight.setPower(-ACTUATOR_SPEED);
                        linearActuatorLeft.setPower(-ACTUATOR_SPEED);
                    } else {
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
                    }*/


                    // exit MANUAL_OVERRIDE
                    if (gamepad2.dpad_down && !checkGTwoDDOWN && gamepad2.a && !checkGTwoA) {
                        checkGTwoDDOWN = true;
                        checkGTwoA = true;

                        overridePID = false;

                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_RETRACT;
                    }

                    break;
                }

            // reset the pivot manually in case of disconnect
            // activated by driver 1 (dpad right and button x)
                case PIVOT_MANUAL_RESET:

                    if (!isStateInitialized) {
                        isRunningPivotToPosition = false;
                        overridePID = true;
                        linearPivotRight.setPower(0);
                        linearPivotLeft.setPower(0);
                        linearPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        linearPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
                        duckSpinner.setPower(DUCK_VALUES.spinStop);
                        isStateInitialized = true;
                    }


                    // SLIDES

                    if (isLinearSlideFullyRetracted(limitSwitch)) {
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    } else if (!isArmPositionSet){
                        linearSlideRight.setPower(-0.5);
                        linearSlideLeft.setPower(-0.5);
                    }


                    // PIVOT

                    if (gamepad1.dpad_up) {
                        // go up
                        linearPivotRight.setPower(0.2);
                        linearPivotLeft.setPower(0.2);
                    } else if (gamepad1.dpad_down) {
                        // go down
                        linearPivotRight.setPower(-0.2);
                        linearPivotLeft.setPower(-0.2);
                    } else {
                        // stop
                        linearPivotRight.setPower(0);
                        linearPivotLeft.setPower(0);
                    }


                    // EXIT/ABORT

                    // go back to automatic function starting from INTAKE_ACTIVE
                    if (gamepad1.dpad_right && !checkGOneDRIGHT && gamepad1.x && !checkGOneX) {
                        checkGOneDRIGHT = true;
                        checkGOneX = true;

                        overridePID = false;
                        linearPivotRight.setPower(0);
                        linearPivotLeft.setPower(0);

                        linearPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearPivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
                    }

                    break;

            
               /* case PIVOT_TO_STABILIZE_ROBOT:
    
                    if (!isStateInitialized) {
                        if (!disableDuck) duckSpinner.setPower(DUCK_VALUES.spinStop);

                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);

                        linearPivotTargetPosition = (int) PIVOT_CONSTANTS.stabilizeReady;
                        isRunningPivotToPosition = true;
                        overridePID = false;
                        // pidTimer.reset();
    
                        lightTimer.reset();
                        // isIntakeProtected = true;
                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }
    
                    // stop slides once finished retracting
                    // (slides started retracting in INTAKE_RETRACT or DEPOSIT_RETRACT)
                    if (isLinearSlideFullyRetracted(limitSwitch)) {
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    } else if (!isArmPositionSet) {
                        linearSlideRight.setPower(-SLIDE_SPEED);
                        linearSlideLeft.setPower(-SLIDE_SPEED);
                    }
    
                    // EXIT
    
                    // make deposit accessible once lift has finished pivoting
                    if (Math.abs(linearPivotAvgPosition - linearPivotTargetPosition) < 20) {
                        checkGTwoDUP = true;
                        isStateInitialized = false;
                        linearPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        if(isExitingHangTime) {
                            // Pivot back to intake (uses deposit to reduce code written)
                            isExitingHangTime = false; // Not needed for going to intake, so we end it here.
                            isGoingToHangTime = false; // Don't accidentally retract back to hanging!!
                            linearSlideState = LinearSlideStates.DEPOSIT_RETRACT; // Put the arm down before going to intake 
                        } else {
                            linearSlideState = LinearSlideStates.STABILIZE_ROBOT; // Get the robot vertical
                        }
                    }
    
                    // ABORT
                    // go back to intake if mistaken
                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        isStateInitialized = false;
                        linearPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                    }
                    break;*/

                /*case STABILIZE_ROBOT:
                    // Put the robot into a vertical position with the arm at the ready
                    final double t = lightTimer.seconds();
                    
                    ascentStabilizer.update(t);
                    final double currentArmTheta = pivotTicksToRadians(linearPivotAvgPosition);
                    final double currentArmLength = liftTicksToHookDistInches(linearSlideAvgPosition);
    
                    if(!isStateInitialized) {
                        if (!disableDuck) duckSpinner.setPower(DUCK_VALUES.spinRest);

                        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);

                        lightTimer.reset();
                        isRunningPivotToPosition = true;
                        overridePID = false;
                        
                        desiredArmTheta = ascentStabilizer.theta(t);
                        desiredArmLength = ascentStabilizer.l(t) - STABILIZER_CONSTANTS.drawBack;
                        
                        // linearSlideLeft.setTargetPosition((int) hookDistInchesToLiftTicks(desiredArmLength));
                        // linearSlideRight.setTargetPosition((int) hookDistInchesToLiftTicks(desiredArmLength));
                        // linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        // linearSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        // linearSlideLeft.setPower(1.0);
                        // linearSlideRight.setPower(1.0);
                        // linearPivotTargetPosition = (int) radiansToPivotTicks(desiredArmTheta);
                        linearPivotTargetPosition = (int) linearPivotAvgPosition;
                        isStateInitialized = true;
                    }
    
                    // Moving the arm into position
                    // The arm first moves to position length-wise
                    boolean isCorrectLength = true && gamepad2.dpad_up;

                    if(gamepad2.dpad_up && Math.abs(linearSlideAvgPosition - hookDistInchesToLiftTicks(desiredArmLength + 4 * STABILIZER_CONSTANTS.hookRadius)) > 30) {
                        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
                        linearSlideLeft.setTargetPosition((int) hookDistInchesToLiftTicks(desiredArmLength + 4 * STABILIZER_CONSTANTS.hookRadius));
                        linearSlideRight.setTargetPosition((int) hookDistInchesToLiftTicks(desiredArmLength + 4 * STABILIZER_CONSTANTS.hookRadius));
                        linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); // NOTE: May be optional for-- or impeding on-- Java
                        linearSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); // NOTE: May be optional for-- or impeding on-- Java
                        linearSlideLeft.setPower(1.0);
                        linearSlideRight.setPower(1.0);
                        isCorrectLength = false;
                        // isRunningPivotToPosition = false;
                    }

                    // The arm only pivots when the arm is in the correct position
                    boolean isCorrectTheta = isCorrectLength;
                    
                    if(isCorrectLength && Math.abs(linearPivotAvgPosition - radiansToPivotTicks(desiredArmTheta)) > 30) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isRunningPivotToPosition = true;
                        linearPivotTargetPosition = (int) radiansToPivotTicks(desiredArmTheta);
                        isCorrectTheta = false;
                    }

                    boolean isTrueCorrectLength = isCorrectLength && isCorrectTheta;

                    if(isCorrectLength && isCorrectTheta && Math.abs(linearSlideAvgPosition - hookDistInchesToLiftTicks(desiredArmLength)) > 30) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        linearSlideLeft.setTargetPosition((int) hookDistInchesToLiftTicks(desiredArmLength));
                        linearSlideRight.setTargetPosition((int) hookDistInchesToLiftTicks(desiredArmLength));
                        linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); // NOTE: May be optional for-- or impeding on-- Java
                        linearSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); // NOTE: May be optional for-- or impeding on-- Java
                        linearSlideLeft.setPower(1.0);
                        linearSlideRight.setPower(1.0);
                        isTrueCorrectLength = false;
                        // isRunningPivotToPosition = false;
                    }
    
                    if (gamepad2.dpad_down) {
                        linearActuatorLeft.setPower(-ACTUATOR_SPEED);
                        linearActuatorRight.setPower(-ACTUATOR_SPEED);
                    } else {
                        // FIXME: not here originally
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
                    }

                    telemetry.addLine("----- Stabilize Robot -----");
                    telemetry.addData("t", t);
                    telemetry.addData("y", ascentStabilizer.y(t));
                    telemetry.addData("deltaY", ascentStabilizer.deltaY(t));
                    telemetry.addData("desiredArmTheta", desiredArmTheta);
                    telemetry.addData("desiredArmLength", desiredArmLength);
    
                    telemetry.addLine("");
                    telemetry.addData("current armTheta", currentArmTheta);
                    telemetry.addData("current length", currentArmLength);
    
                    // EXIT
                    if((isCorrectLength && isCorrectTheta && isTrueCorrectLength)*//*  || (gamepad2.dpad_up && !checkGTwoDUP) *//*) {
                        if(gamepad2.dpad_up) {
                            checkGTwoDUP = true;
                        }

                        // FIXME: not here originally
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
    
                        linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearSlideLeft.setPower(0);
                        linearSlideRight.setPower(0);
                        linearPivotLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearPivotLeft.setPower(0);
                        linearPivotRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearPivotRight.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.HANG_TIME_AUTOMATIC_HANDS;
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);

                    }
    
                    // ABORT
                    // Move the arm out of the way and then retract the arm if mistaken.
                    if(gamepad2.dpad_right && !checkGTwoDRIGHT) {

                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);

                        linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearSlideLeft.setPower(0);
                        linearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearSlideRight.setPower(0);
                        linearPivotLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearPivotLeft.setPower(0);
                        linearPivotRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearPivotRight.setPower(0);
                        checkGTwoDRIGHT = true;
                        isStateInitialized = false;
                        isExitingHangTime = true; // This flag MUST be used if the exiting is to be done
                        linearSlideState = LinearSlideStates.PIVOT_TO_STABILIZE_ROBOT;
                    }
                    break;*/
    
                /*case HANG_TIME_AUTOMATIC_HANDS:
                    // Vertically raising the robot for the first portion of a 3rd level ascent
                    //
                    // This uses RUN_TO_POSITION to maintain the position of the motors; this may,
                    // however, overpull the robot due to the velocities being not maintained. You 
                    // can attempt to restabilize the robot by hitting the right dpad button, the effect
                    // might be lackluster
                    final double t = lightTimer.seconds();
                    
                    ascentStabilizer.update(t);
                    
                    // Raise the robot vertically to 3rd level!
                    if(!isStateInitialized) {
                        lightTimer.reset();
                        isRunningPivotToPosition = true;
                        overridePID = false;
                        ascentStabilizer.firstUpdate = true;
                        ascentStabilizer.update(t);
                        isStateInitialized = true;
                    }
    
                    desiredArmTheta = ascentStabilizer.theta(t);
                    desiredArmLength = ascentStabilizer.l(t) - STABILIZER_CONSTANTS.drawBack;
    
                    linearSlideLeft.setTargetPosition((int) hookDistInchesToLiftTicks(desiredArmLength));
                    linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    linearSlideLeft.setPower(1.0);
                    linearSlideRight.setTargetPosition((int) hookDistInchesToLiftTicks(desiredArmLength));
                    linearSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    linearSlideRight.setPower(1.0);
    
                    linearPivotTargetPosition = (int) radiansToPivotTicks(desiredArmTheta);
    
                    if (gamepad2.dpad_down) {
                        linearActuatorLeft.setPower(-ACTUATOR_SPEED);
                        linearActuatorRight.setPower(-ACTUATOR_SPEED);
                    } else {
                        // FIXME: not here originally
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
                    }

                    telemetry.addLine("--- Automatic Hang Time (Alternate ~ Position) ---");
                    telemetry.addData("t", t);
                    telemetry.addData("y", ascentStabilizer.y(t));
                    telemetry.addData("deltaY", ascentStabilizer.deltaY(t));
                    telemetry.addData("desiredArmTheta", desiredArmTheta);
                    telemetry.addData("desiredArmLength", desiredArmLength);
                    telemetry.addData("currentArmLift", liftTicksToHookDistInches(linearSlideAvgPosition));
                    telemetry.addData("currentArmTheta", pivotTicksToRadians(linearPivotAvgPosition));
    
                    // EXIT
                    // Go into manual hang mode 
                    if(gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;

                        // FIXME: not here originally
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);

                        linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearSlideLeft.setPower(0);
                        linearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearSlideRight.setPower(0);
                        linearPivotLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearPivotLeft.setPower(0);
                        linearPivotRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearPivotRight.setPower(0);
                        isStateInitialized = false;
                        
                        isRunningPivotToPosition = true;
                        overridePID = false;
                        linearSlideState = LinearSlideStates.HANG_TIME_AUTOMATIC_ARM;
                    }
    
                    // Restabilize the robot to remove additive error from adding only *velocities*
                    if(gamepad2.dpad_right && !checkGTwoDRIGHT) {
                        checkGTwoDRIGHT = true;

                        // FIXME: not here originally
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);

                        linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearSlideLeft.setPower(0);
                        linearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearSlideRight.setPower(0);
                        linearPivotLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearPivotLeft.setPower(0);
                        linearPivotRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearPivotRight.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.STABILIZE_ROBOT;
                    }
                    break;*/
                
                
                /*// Keeping the robot vertical while manually changing the length of the arm
                case HANG_TIME_AUTOMATIC_ARM:
                    if(!isStateInitialized) {
                        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isRunningPivotToPosition = true;
                        overridePID = false;
                        isStateInitialized = true;
                    }
                    
                    // Powering the arm
                    linearSlidePower = calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.HANG);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);

                    // Getting the values for the angle
                    final double t = lightTimer.seconds();
                    ascentStabilizer.update(t);
                    desiredArmLength = liftTicksToHookDistInches(linearSlideAvgPosition);
                    desiredArmTheta = ascentStabilizer.thetaFromL(desiredArmLength);
                    linearPivotTargetPosition = (int) radiansToPivotTicks(desiredArmTheta);

                    // EXIT
                    // Transition to the next phase
                    if(gamepad2.dpad_up && checkGTwoDUP) {
                        checkGTwoDUP = true;
                        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isRunningPivotToPosition = true;
                        overridePID = false;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.HANG_TIME_MANUAL;
                    }
                    break;*/

                
                /*// Manually change the angle and length of the arm
                case HANG_TIME_MANUAL:
                    if(!isStateInitialized) {
                        linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearSlideLeft.setPower(0);
                        linearSlideRight.setPower(0);

                        linearPivotLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearPivotRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        linearPivotLeft.setPower(0);
                        linearPivotRight.setPower(0);
                        
                        isRunningPivotToPosition = false;
                        overridePID = true;
                        isStateInitialized = true;
                    }

                    linearSlidePower = calculateSlidePower(linearSlideAvgPosition, ExtensionLimits.HANG);
                    linearSlideRight.setPower(linearSlidePower);
                    linearSlideLeft.setPower(linearSlidePower);

                    double linearPivotPower = calculateManualPivotPower(linearPivotAvgPosition);
                    linearPivotRight.setPower(linearPivotPower);
                    linearPivotLeft.setPower(linearPivotPower);

                    // EXIT
                    // Move the arm out of the way and then retract the arm.
                    // Code disabled to allow to prevent accidental dropping!!
                    *//* if(gamepad2.dpad_right && !checkGTwoDRIGHT) {
                         linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                         linearSlideLeft.setPower(0);
                         linearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                         linearSlideRight.setPower(0);
                         linearPivotLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                         linearPivotLeft.setPower(0);
                         linearPivotRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                         linearPivotRight.setPower(0);
                         checkGTwoDRIGHT = true;
                         isRunningPivotToPosition = true;
                         overridePID = false;
                         isStateInitialized = false;
                         isExitingHangTime = true; // This flag MUST be used if the exiting is to be done
                         linearSlideState = LinearSlideStates.PIVOT_TO_STABILIZE_ROBOT;
                     }*//*
                    break;*/

                /* case HANG_TIME_AUTOMATIC_ARM_ALTERNATE: {
                     if(!isStateInitialized) {
                         isStateInitialized = true;
                     }

                     // EXIT
                     if(gamepad2.dpad_up && !checkGTwoDUP) {
                         isStateInitialized = false;
                         linearSlideState = LinearSlideStates.HANG_TIME_MANUAL;
                     }
                     break;
                 }

                 case HANG_TIME_AUTOMATIC_ARM_ALTERNATE: {
                     if(!isStateInitialized) {
                         isStateInitialized = true;
                     }

                     // EXIT
                     if(gamepad2.dpad_up && !checkGTwoDUP) {
                         isStateInitialized = false;
                         linearSlideState = LinearSlideStates.HANG_TIME_MANUAL;
                     }
                     break;
                 } */
            } // end lift state machine

            // run position power controller for pivot
            if (isRunningPivotToPosition) {
                runPivotToPositionIndividual(
                        linearPivotRight.getCurrentPosition(),
                        linearPivotLeft.getCurrentPosition(),
                        linearPivotTargetPosition,
                        pivotPIDSpeedMultiplier);
            } else if (!overridePID) {
                linearPivotRight.setPower(0);
                linearPivotLeft.setPower(0);
            }

// LINEAR ACTUATORS ------------------------------------------------------------------------

            /*// used in combination with stored and running time for actuators
            double handTime = 0; // pretty sure this variable is useless

            switch (actuatorHangState) {
                // moves hands up to correct position upon initialization
                // and whenever hands are reset
                case HANDS_INITIALIZE:
                    if (!isActuatorStateInitialized) {
                        linearActuatorRight.setPower(ACTUATOR_SPEED);
                        linearActuatorLeft.setPower(ACTUATOR_SPEED);
                        storedActuatorTime = 0;
                        actuatorTimer.reset();
                        isRobot2ndLevelAscending = false;
                        isActuatorStateInitialized = true;
                    }

                    linearActuatorRight.setPower(ACTUATOR_SPEED);
                    linearActuatorLeft.setPower(ACTUATOR_SPEED);

                    // EXIT

                    // go to rest
                    if (actuatorTimer.seconds() > TIME_CONSTANTS.handsInit) {
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
                        isActuatorStateInitialized = false;
                        actuatorHangState = ActuatorHangStates.HANDS_AT_REST;
                    }

                    // reset hands
                    if (gamepad1.dpad_left) {
                        isActuatorStateInitialized = false;
                        actuatorHangState = ActuatorHangStates.HANDS_RESET;
                    }
                    break;


            // move hands up to ready to hang
                case HANDS_UP:
                    if (!isActuatorStateInitialized) {
                        linearActuatorRight.setPower(ACTUATOR_SPEED);
                        linearActuatorLeft.setPower(ACTUATOR_SPEED);
                        actuatorTimer.reset();
                        isActuatorStateInitialized = true;
                    }

                    linearActuatorRight.setPower(ACTUATOR_SPEED);
                    linearActuatorLeft.setPower(ACTUATOR_SPEED);

                    handTime = actuatorTimer.seconds() + storedActuatorTime;

                    // EXIT

                    // stop hands once they have presumably gone up far enough
                    if (handTime > TIME_CONSTANTS.handsUp) {
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
                        isActuatorStateInitialized = false;
                        actuatorHangState = ActuatorHangStates.HANDS_AT_REST;
                    }

                    // ABORT

                    // go to hands down if canceled
                    *//*if (gamepad1.dpad_up && !checkGOneDUP) {
                        checkGOneDUP = true;
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
                        storedActuatorTime = handTime;
                        isRobot2ndLevelAscending = false; // going down!
                        isActuatorStateInitialized = false;
                        actuatorHangState = ActuatorHangStates.HANDS_DOWN;
                    }*//*

                    // reset hands
                    if (gamepad1.dpad_left) {
                        isActuatorStateInitialized = false;
                        actuatorHangState = ActuatorHangStates.HANDS_RESET;
                    }
                    break;

            // move hands down to hang
                case HANDS_DOWN:
                    if (!isActuatorStateInitialized) {
                        linearActuatorRight.setPower(-ACTUATOR_SPEED);
                        linearActuatorLeft.setPower(-ACTUATOR_SPEED);
                        actuatorTimer.reset();
                        isActuatorStateInitialized = true;
                    }

                    linearActuatorRight.setPower(-ACTUATOR_SPEED);
                    linearActuatorLeft.setPower(-ACTUATOR_SPEED);

                    handTime = actuatorTimer.seconds() + storedActuatorTime;

                    // EXIT

                    // stop hands once they have presumably gone down far enough
                    if (handTime > TIME_CONSTANTS.handsDown) {
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
                        isActuatorStateInitialized = false;
                        actuatorHangState = ActuatorHangStates.HANDS_AT_REST;
                    }

                    // ABORT

                    // go to hands up if canceled
                    *//*if (gamepad1.dpad_up && !checkGOneDUP) {
                        checkGOneDUP = true;
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
                        storedActuatorTime = handTime;
                        isRobot2ndLevelAscending = false; // going up!
                        isActuatorStateInitialized = false;
                        actuatorHangState = ActuatorHangStates.HANDS_UP;
                    }*//*

                    // reset hands
                    if (gamepad1.dpad_left) {
                        isActuatorStateInitialized = false;
                        actuatorHangState = ActuatorHangStates.HANDS_RESET;
                    }
                    break;


                case HANDS_AT_REST:
                    if (!isActuatorStateInitialized) {
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
                        storedActuatorTime = 0;
                        isActuatorStateInitialized = true;
                    }

                    // ACTUATORS

                    // move down to adjust slightly if timers did not get it right
                    if (gamepad1.back) {
                        linearActuatorRight.setPower(-ACTUATOR_SPEED);
                        linearActuatorLeft.setPower(-ACTUATOR_SPEED);
                    } else {
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
                    }

                    // EXIT

                    // currently set up with driver 1 controls as backup

                    // move hands to and from hanging
                    if (gamepad1.dpad_up && !checkGOneDUP) {
                        checkGOneDUP = true;

                        if (!isRobot2ndLevelAscending) {
                            isRobot2ndLevelAscending = true;
                            isActuatorStateInitialized = false;
                            actuatorHangState = ActuatorHangStates.HANDS_DOWN;
                        } else {
                            isRobot2ndLevelAscending = false;
                            isActuatorStateInitialized = false;
                            actuatorHangState = ActuatorHangStates.HANDS_UP;
                        }
                    }

                    // ABORT

                    // reset hands
                    if (gamepad1.dpad_left) {
                        isActuatorStateInitialized = false;
                        actuatorHangState = ActuatorHangStates.HANDS_RESET;
                    }
                    break;


                case HANDS_RESET:
                    if (!isActuatorStateInitialized) {
                        isRobot2ndLevelAscending = false;
                        linearActuatorRight.setPower(-ACTUATOR_SPEED);
                        linearActuatorLeft.setPower(-ACTUATOR_SPEED);
                        isActuatorStateInitialized = true;
                    }

                    linearActuatorRight.setPower(-ACTUATOR_SPEED);
                    linearActuatorLeft.setPower(-ACTUATOR_SPEED);

                    // EXIT
                    // stop moving actuators down once button is let go
                    // (goes back to initialization stage)
                    if (!gamepad1.dpad_left) {
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
                        isActuatorStateInitialized = false;
                        actuatorHangState = ActuatorHangStates.HANDS_INITIALIZE;
                    }
                    break;
            } // end actuator switch statement*/




// DRIVETRAIN ------------------------------------------------------------------------------------


            // set drive speed
            if (gamepad1.right_bumper && !checkGOneRB && driveSpeedIndex < driveSpeedRange.length-1) {
                driveSpeedIndex++;
                driveSpeedFactor = driveSpeedRange[driveSpeedIndex];
                checkGOneRB = true;

            } else if (gamepad1.left_bumper && !checkGOneLB && driveSpeedIndex > 0) {
                driveSpeedIndex--;
                driveSpeedFactor = driveSpeedRange[driveSpeedIndex];
                checkGOneLB = true;
            }

            // check for AprilTag detections if told to do so
            // runs to a tag if it found one, otherwise normal drivetrain control
            // NOTE: button must be held to auto drive to tag
            if (gamepad1.y && camera) {
                runningToBucketAprilTag = glasses.detectAprilTag(-1);
            } else {
                runningToBucketAprilTag = false;
            }

            // select correct drive and set power
            if (runningToBucketAprilTag) {
                // automatically position drivetrain to place in top bucket

                glasses.driveToAprilTag(SENSOR_VARIABLES.glassesDistance,
                        SENSOR_VARIABLES.glassesBearing,
                        SENSOR_VARIABLES.glassesYaw,
                        SENSOR_VARIABLES.glassesCameraAngle);

            } else if (tankDrive) {
                // tank mecanum drive

                rightPower = -gamepad1.right_stick_y * driveSpeedFactor;
                leftPower = -gamepad1.left_stick_y * driveSpeedFactor;

                slideToRight = -gamepad1.right_trigger * driveSpeedFactor * 1.1;
                slideToLeft = -gamepad1.left_trigger * driveSpeedFactor * 1.1;

                double tankDenominator = Math.max(Math.abs((rightPower+leftPower)/2) + Math.abs(slideToRight) + Math.abs(slideToLeft), 1);

                frontRight.setPower((rightPower + slideToRight - slideToLeft) / tankDenominator);
                backRight.setPower((rightPower - slideToRight + slideToLeft) / tankDenominator);
                frontLeft.setPower((leftPower - slideToRight + slideToLeft)/ tankDenominator);
                backLeft.setPower((leftPower + slideToRight - slideToLeft)/ tankDenominator);

            } else {
                // standard mecanum drive

                powerX = gamepad1.left_stick_x * 1.1;
                powerY = -gamepad1.left_stick_y;
                powerRX = gamepad1.right_stick_x;

                double mecanumDenominator = Math.max(Math.abs(powerX) + Math.abs(powerY) + Math.abs(powerRX), 1);

                frontRight.setPower(((powerY - powerX - powerRX) / mecanumDenominator)*driveSpeedFactor);
                backRight.setPower(((powerY + powerX - powerRX) / mecanumDenominator)*driveSpeedFactor);
                frontLeft.setPower(((powerY + powerX + powerRX) / mecanumDenominator)*driveSpeedFactor);
                backLeft.setPower(((powerY - powerX + powerRX) / mecanumDenominator)*driveSpeedFactor);
            }




// TELEMETRY ------------------------------------------------------------------------------------
            telemetry.addData("Duck Disabled", disableDuck);
            telemetry.addLine("MANUAL OVERRIDE: (gamepad 2) dpad_down + button_a");
            telemetry.addLine("-------------------------");

            telemetry.addData("SLIDE STATE", linearSlideState);
            telemetry.addData("TIME", lightTimer.seconds());
            telemetry.addLine("-------------------------");

            /*telemetry.addLine("ACTUATORS");
            telemetry.addData("ACTUATOR STATE", actuatorHangState);
            telemetry.addData("LAR POW", linearActuatorRight.getPower());
            telemetry.addData("LAL POW", linearActuatorLeft.getPower());
            telemetry.addLine("-------------------------");*/

            telemetry.addLine("DRIVETRAIN");
            telemetry.addData("Running to AprilTag", runningToBucketAprilTag);
            telemetry.addData("Front R POW", frontRight.getPower());
            telemetry.addData("Back R POW", backLeft.getPower());
            telemetry.addData("Front R POW", frontRight.getPower());
            telemetry.addData("Back L POW", backLeft.getPower());
            telemetry.addData("Drive Speed Factor", driveSpeedFactor);
            telemetry.addLine("-------------------------");

            telemetry.addLine("LIFT SLIDES");
            telemetry.addData("Slide R POW", linearSlideRight.getPower());
            telemetry.addData("Slide L POW", linearSlideLeft.getPower());
            telemetry.addData("Slide R POS", linearSlideRight.getCurrentPosition());
            telemetry.addData("Slide L POS", linearSlideLeft.getCurrentPosition());
            telemetry.addData("Slide AVG POS", linearSlideAvgPosition);
            telemetry.addData("Slide AVG Extension (in)", liftTicksToInches(linearSlideAvgPosition));
            telemetry.addData("Hook Distance", liftTicksToHookDistInches(linearSlideAvgPosition));
            telemetry.addLine("-------------------------");

            telemetry.addLine("LIFT PIVOTS");
            telemetry.addData("Pivot R POW", linearPivotRight.getPower());
            telemetry.addData("Pivot L POW", linearPivotLeft.getPower());
            telemetry.addData("Pivot R POS", linearPivotRight.getCurrentPosition());
            telemetry.addData("Pivot L POS", linearPivotLeft.getCurrentPosition());
            telemetry.addData("Pivot AVG POS", linearPivotAvgPosition);
            telemetry.addData("Pivot AVG Extension (deg)", 360 / (2 * Math.PI) * pivotTicksToRadians(linearPivotAvgPosition));
            telemetry.addLine("-------------------------");

            telemetry.addLine("SERVOS");
            telemetry.addData("Intake WR POW", intakeWheelR.getPower());
            telemetry.addData("Intake WL POW", intakeWheelL.getPower());
            telemetry.addData("Intake PIVOT POS", intakePivot.getPosition());
            telemetry.addData("Specimen POS", specimenGrabber.getPosition());
            telemetry.addLine("-------------------------");

            telemetry.addLine("SENSORS");
//            telemetry.addData("Height Sensor Dist (in)", heightSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Height Sensor Getter (in)", heightGetter.getDistance(DistanceUnit.INCH));
            telemetry.addData("Limit Switch Activated", linearSlideSwitch.isPressed());
            telemetry.addData("Sample Sensor Gain", sampleSensor.getGain());
            telemetry.addData("Sample DIST (CM)", sampleSensor.getDistance(DistanceUnit.CM));
            telemetry.addLine("(operating range 1-10 centimeters)");
            telemetry.addData("Red", sampleSensor.getNormalizedColors().red);
            telemetry.addData("Green", sampleSensor.getNormalizedColors().green);
            telemetry.addData("Blue", sampleSensor.getNormalizedColors().blue);
            telemetry.addLine("-------------------------");

            telemetry.addLine("LOGIC");
            telemetry.addData("Specimanning", specimanning);
//            telemetry.addData("Hanging", isGoingToHangTime);
            telemetry.addLine("-------------------------");


            if (camera) {
                if (glasses.hasAprilTag()) {
                    telemetry.addLine("VISION");
                    glasses.addAprilTagTelemetry();
                }
            }

            telemetry.update();


        }
    } // end run opmode method


    /**
     * Used in automation. Determines possession based on distance.
     * @param currentSampleDistance current distance reading from the sample sensor
     * @return whether intake can see it has collected a sample
     **/
    private boolean isPossessingSample(double currentSampleDistance) {
        // TODO: add color sensor function here

        // robot has successfully acquired a sample
        if (currentSampleDistance < SENSOR_VARIABLES.sampleDistance) {
            return true;
        } else {
            // robot did not get sample
            return false;
        }
    }


    /**
     * Used in automation. Can be overridden with a button press.
     * @param limitSwitch current status of magnetic limit switch
     * @return whether slide hit the limit switch to zero out
     */
    private boolean isLinearSlideFullyRetracted(boolean limitSwitch) {
        return limitSwitch || gamepad2.x || gamepad2.y;
    }



    /**
     * Determines what power to apply to the slides specified by the mode the lift is in. <br>
     * Utilizes a cushion effect to prevent over-extension.
     * @param linearSlidePosition current position of the slide
     * @param limit which extension limit cushion to use
     * @return a correct and safe power that can be applied to the linear slide motor
     */
    private double calculateSlidePower(double linearSlidePosition, ExtensionLimits limit) {
        double linearSlidePower = 0;
        double linearSlideCushion = 1;

        // reset acceleration timer
        // used for smooth movement
//        if ((-gamepad2.right_stick_y) < 0.01 && (-gamepad2.right_stick_y) > -0.01) slideTimer.reset();

        // determine the cushion for the linear slide so robot does not exceed extension limit
        switch (limit) {
            case INTAKE:
                linearSlideCushion = (SLIDE_CONSTANTS.extensionLimitIntake - linearSlidePosition)
                        /SLIDE_CONSTANTS.cushionRatio;
                break;

            case HANG:
            case DEPOSIT:
                linearSlideCushion = (SLIDE_CONSTANTS.topBucketHeightAlternate - linearSlidePosition)
                        /SLIDE_CONSTANTS.cushionRatio;
                break;

            case SPECIMEN:
                linearSlideCushion = (SLIDE_CONSTANTS.extensionLimitSpecimen - linearSlidePosition)
                        /SLIDE_CONSTANTS.cushionRatio;
                break;

            case WALL:
                linearSlideCushion = (SLIDE_CONSTANTS.extensionLimitWall - linearSlidePosition)
                        /SLIDE_CONSTANTS.cushionRatio;
                break;
        }


        // determine whether to apply the cushion or ignore it
        if ((-gamepad2.right_stick_y) > 0) {
            // lift going up, use cushion
            linearSlidePower = (-gamepad2.right_stick_y)*SLIDE_SPEED*linearSlideCushion;
        } else {
            // lift going down, ignore cushion
            linearSlidePower = (-gamepad2.right_stick_y)*SLIDE_SPEED;
        }

        // for deposit only
        if (limit == ExtensionLimits.DEPOSIT || limit == ExtensionLimits.SPECIMEN) {
            // make descent of slides slightly slower so it is not jarring
            if ((-gamepad2.right_stick_y) < 0) {
                linearSlidePower*=0.9;
            }
            // apply a coefficient to fight gravity (slide holds power to extend)
            linearSlidePower+=SLIDE_CONSTANTS.gravityCoefficient;
        }

        if (limit == ExtensionLimits.HANG) {
            // make it so the robot doesn't drop itself when trying to adjust height
            if ((-gamepad2.right_stick_y) > 0) {
                linearSlidePower*=0.5;
            }
            // apply a coefficient to fight gravity (slide holds power to retract)
            linearSlidePower-=SLIDE_CONSTANTS.gravityCoefficient;
        }

        // calculate acceleration
//        double accelerationDampener = Range.clip((slideTimer.seconds()*2), 0, 1.0);
//        linearSlidePower*=accelerationDampener;

        return linearSlidePower;
    }



    /**
     * Prevents pivot from bending the robot in half and exceeding the extension limit <br>
     * when running the pivot manually
     * @param pivotPosition current position of the pivot
     * @return a power suitable for safe pivot use
     */
    private double calculateManualPivotPower(double pivotPosition) {
        double pivotPower = 0;
        double pivotCushion = 1;

        // find pivot cushion so it does not bend the robot in half
        if ((-gamepad2.left_stick_y) > 0) {
            // for moving pivot up
            // END POSITION MAX
            pivotCushion = ((double)PIVOT_CONSTANTS.maxPos - pivotPosition)/PIVOT_CONSTANTS.cushionRatio;

        } else {
            // for moving pivot down
            // END POSITION 0
            pivotCushion = (pivotPosition - (double)PIVOT_CONSTANTS.minPos)/PIVOT_CONSTANTS.cushionRatio;

        }

        // find feedforward to fight gravity based on arm being completely horizontal when starting
        double pivotFF = Math.cos(Math.toRadians(pivotPosition / PIVOT_TICKS_PER_DEGREE + 1)) * PIVOT_CONSTANTS.gravityFeedForward;

        // set power
        pivotPower = (-gamepad2.left_stick_y)*(PIVOT_SPEED*0.3)*pivotCushion;

        // apply a factor to fight gravity if needed (works and doesn't work at the same time)
        // note that this does not account for the added load due to lever action
//        if (pivotPosition > 100) {
//            pivotPower += pivotFF;
//        }
        pivotPower += pivotFF;

        return pivotPower;
    }


    /**
     * Runs and sets pivot motor powers based on a target position while fighting gravity. <br>
     * <strong>This should be used in place of RUN_TO_POSITION for the chain driven pivots!</strong> <br>
     * (note when not in use, powers should manually be set to 0 to avoid residual motor power.)
     * @param currentPositionR current position of right pivot motor
     * @param currentPositionL current position of left pivot motor
     * @param targetPosition desired end position for lift pivot
     **/
    private void runPivotToPositionIndividual(int currentPositionR, int currentPositionL, int targetPosition, double speedMultiplier) {
        // update PID variables every loop
        pivotController.setPID(PIVOT_CONSTANTS.kp, PIVOT_CONSTANTS.ki, PIVOT_CONSTANTS.kd);

        // calculate power to apply to pivots
        double pivotPIDR = pivotController.calculate(currentPositionR, targetPosition);
        pivotPIDR = Range.clip(pivotPIDR, -1.0, 1.0);

        double pivotPIDL = pivotController.calculate(currentPositionL, targetPosition);
        pivotPIDL = Range.clip(pivotPIDL, -1.0, 1.0);

        // find feedforward to fight gravity based on arm being completely horizontal when starting
        double pivotFFR = Math.cos(Math.toRadians(currentPositionR / PIVOT_TICKS_PER_DEGREE + 1)) * PIVOT_CONSTANTS.gravityFeedForward;
        double pivotFFL = Math.cos(Math.toRadians(currentPositionL / PIVOT_TICKS_PER_DEGREE + 1)) * PIVOT_CONSTANTS.gravityFeedForward;

        // perhaps a more useful calculation if position is wanted to be found when arm is vertically down
//            ff = Math.sin(Math.toRadians(armPos / ticks_in_degree + zeroOffset )) * f;

//        double pivotPower = (pivotPID * PIVOT_CONSTANTS.speedCap) + pivotFF;

        // modify raw PID power so robot does not rattle apart since kd was not very effective
        double accelerationFactor = Range.clip((pidTimer.seconds()*10), 0, 1.0);
        double acceleratedPowerR = Math.min(accelerationFactor*pivotPIDR, PIVOT_SPEED);
        double acceleratedPowerL = Math.min(accelerationFactor*pivotPIDL, PIVOT_SPEED);
        double pivotPowerR = acceleratedPowerR + pivotFFR;
        double pivotPowerL = acceleratedPowerL + pivotFFL;

        pivotPowerR*=speedMultiplier;
        pivotPowerL*=speedMultiplier;

        // apply power to pivot motors
        linearPivotRight.setPower(pivotPowerR);
        linearPivotLeft.setPower(pivotPowerL);
    }


    /**
     * Initializes all hardware needed to begin teleop
     */
    private void initHardware() {

        // HARDWARE CONFIGURATION
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        linearSlideRight = hardwareMap.get(DcMotorEx.class, "linearSlideRight");
        linearSlideLeft = hardwareMap.get(DcMotorEx.class, "linearSlideLeft");

        linearPivotRight = hardwareMap.get(DcMotorEx.class, "linearPivotRight");
        linearPivotLeft = hardwareMap.get(DcMotorEx.class, "linearPivotLeft");

//        linearActuatorRight = hardwareMap.get(CRServo.class, "linearActuatorRight");
//        linearActuatorLeft = hardwareMap.get(CRServo.class, "linearActuatorLeft");

        intakeWheelR = hardwareMap.get(CRServo.class, "intakeWheelR");
        intakeWheelL = hardwareMap.get(CRServo.class, "intakeWheelL");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        duckSpinner = hardwareMap.get(CRServo.class, "duckSpinner");

        specimenGrabber = hardwareMap.get(Servo.class, "specimenGrabber");

        sampleSensor = hardwareMap.get(ColorRangeSensor.class, "sampleSensor");
        linearSlideSwitch = hardwareMap.get(TouchSensor.class, "linearSlideSwitch");

//        imu = hardwareMap.get(IMU.class, "imu");
//        heightSensor = hardwareMap.get(DistanceSensor.class, "heightSensor");

        // MOTOR/SERVO DIRECTIONS AND POSITION INITIALIZATION
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        linearPivotRight.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearPivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sampleSensor.setGain(SENSOR_VARIABLES.sampleSensorGain);

        intakeWheelR.setDirection(DcMotorSimple.Direction.REVERSE);

        // normally enabled
        if (camera) {
            glasses = new RobotVision(hardwareMap, telemetry, true, false);

        }

    }

    private double pivotTicksToRadians(double ticks) {
        return ticks / PIVOT_TICKS_PER_RAD;
    }

    private double liftTicksToInches(double ticks) {
        return ticks / LIFT_TICKS_PER_INCH_EXTENDED;
    }

    private double radiansToPivotTicks(double ticks) {
        return ticks * PIVOT_TICKS_PER_RAD;
    }

    private double inchesToLiftTicks(double inches) {
        return inches * LIFT_TICKS_PER_INCH_EXTENDED;
    }

    private double liftTicksToHookDistInches(double ticks) {
        return liftTicksToInches(ticks) + STABILIZER_CONSTANTS.initialHookDist;
    }

    private double hookDistInchesToLiftTicks(double inches) {
        return inchesToLiftTicks(inches - STABILIZER_CONSTANTS.initialHookDist);
    }
}
