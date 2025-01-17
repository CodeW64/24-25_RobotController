package org.firstinspires.ftc.teamcode.teamprograms.auto;

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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotVision;

/**
 * Welcome!
 * Teleop Version: 2.2.2 + 2.5.0 AUTO-EDIT
 * STARTING POSITION/STATE: INTAKE_ACTIVE
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
 * [dpad_up] - manually raise pivot by given number of ticks, go to stasis
 * [button_a] - exit manual pivot adjustment (manual pivot stasis)
 * [dpad_down] - manually lower pivot by given number of ticks, go to stasis
 * [button_b] - increase hanging position for pivot slightly (pivot hang)
 * [button_a] - decrease hanging position for pivot slightly (pivot hang)
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
 * [dpad_up] - enter/exit HANG_TIME (from deposit mode)
 * [dpad_down + a_button] - enter MANUAL_OVERRIDE (DANGEROUS)
 * --------------------------
 * MANUAL_OVERRIDE (affects Gamepad 2 only)
 * NOTE: this mode is EXTREMELY DANGEROUS (yes, more than last year)
 *      BE ALERT: if the robot has disconnected, the robot does not know
 *                where the pivot position should be; during this, DO NOT TRY TO EXIT
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

@TeleOp(name = "AutoArmRunner2 Test Suite", group = "A")
@Config
public class AutoArmRunner2 extends LinearOpMode {
    public boolean isTelemetrySuppresed = false;

    // HARDWARE
    protected DcMotorEx frontRight, backRight, frontLeft, backLeft;
    protected DcMotorEx linearSlideRight, linearSlideLeft;
    protected DcMotorEx linearPivotRight, linearPivotLeft;
    protected CRServo linearActuatorRight, linearActuatorLeft;
    protected CRServo intakeWheelR, intakeWheelL;
    protected Servo intakePivot;
    protected Servo specimenGrabber;
    protected CRServo duckSpinner;
    protected DistanceSensor heightSensor;
    protected IMU imu;

    protected ColorRangeSensor sampleSensor;
    protected TouchSensor linearSlideSwitch;
    protected RobotVision glasses;

    // SERVO POSITION VALUES (editable by FTC dashboard)
    public static class ServoValues {
        public double pivotIntakePos = 0.40;
        public double pivotHoverPos = 0.48;
        public double pivotEjectSamplePos = 0.6;
        public double pivotDepositPos = 0.4;
        public double pivotRestPos = 0.52;
        public double pivotCarryPos = 0.6;
        public double specimenGrabberOpenPos = 0.58; // adjust
        public double specimenGrabberClosePos = 0.47; // adjust
    }
    public static ServoValues SERVO_VALUES = new ServoValues();

    // more servo variables
    final double INTAKE_POWER_MAX = 1.0;
    final double INTAKE_POWER_HOLD = 0.06;
    final double INTAKE_POWER_EMPTY = -1.0;
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

    // SLIDE VARIABLES (editable by FTC dashboard)
    public static class SlideConstants {
        public double gravityCoefficient = 0.0005;
        public double extensionLimitIntake = 1780.0; // 312RPM-2500
        public double extensionLimitSpecimen = 2700.0; // 312RPM-3800 // FIXME: adjust to fit within limit
        public double extensionLimitHang = 2700.0; // 312RPM-3800
        public double cushionRatio = 400.0;
        public double topBucketHeightAlternate = 2930.0; // 312RPM-4100
        public double depositEndRetract = 1800;
        public double intakeEndRetract = 800;
    }
    public static SlideConstants SLIDE_CONSTANTS = new SlideConstants();

    // PIVOT VARIABLES (editable by FTC dashboard)
    public static class PivotConstants {
        public double cushionRatio = 400;
        public double kp = 0.005;
        public double ki = 0.0;
        public double kd = 0.0; // 0.0002
        public double gravityFeedForward = 0.002;
        public double retractSetSpeedMultiplier = 1.0;

        public double maxPos = 2000;
        public double minPos = 0;
        public double intakePos = 100;
        public double depositPos = 1650;
        public double depositRetractSetPos = 1800;
        public double specimenGrabPos = 700; // adjust to right angle
        public double specimenPositionPos = 1200; // adjust to right angle
        public double specimenPlacePos = 1000; // adjust to right angle
        public double stabilizeReady = 1300;

    }
    public static PivotConstants PIVOT_CONSTANTS = new PivotConstants();

    public final static double PIVOT_TICKS_PER_DEGREE = 2000.0 / 90.0; // (motor PPR / gear ratio) / 360
    public final static double PIVOT_TICKS_PER_RAD = PIVOT_TICKS_PER_DEGREE * 180 / Math.PI;

    public final double LIFT_TICKS_PER_INCH_EXTENDED = 384.5 / 537.7 * 3450.0 / 30.0;

    private PIDController pivotController;

    // ROBOT LIFT STATES
    public enum LinearSlideStates {

        INTAKE_ACTIVE, INTAKE_ATTEMPT_SAMPLE, INTAKE_DIVIDE_SAMPLE, INTAKE_FULL, INTAKE_EMPTY,
        INTAKE_RETRACT, PIVOT_TO_DEPOSIT,

        DEPOSIT_ACTIVE,
        DEPOSIT_RETRACT_SET, DEPOSIT_RETRACT, PIVOT_TO_INTAKE,
        
        PIVOT_TO_SPECIMEN_GRAB,
        SPECIMEN_GRAB, SPECIMEN_POSITION, SPECIMEN_PLACE,
        SPECIMEN_RETRACT,

        MANUAL_OVERRIDE,

        PIVOT_MANUAL_RESET
    }
    protected LinearSlideStates linearSlideState;
 
    enum ExtensionLimits {
        INTAKE, DEPOSIT, SPECIMEN, HANG
    }

    public ElapsedTime lightTimer, setupTimer, pidTimer, slideTimer;

    // constant variables
    final double SLIDE_SPEED = 1.0;
    final double PIVOT_SPEED = 1.0;
    final double ACTUATOR_SPEED = 1.0;

    boolean tankDrive = true;
    boolean disableDuck = false;
    boolean runningToBucketAprilTag = false;
    boolean overridePID = false;
    boolean camera = false; // disable if camera not in use or if it doesn't exist
    boolean isRunningPivotToPosition = false;
    boolean specimanning = false;
    private boolean fightGravity = true; 

    protected boolean isStateInitialized = false;
    protected double currentSampleDistance = 0;


    protected int linearPivotTargetPosition = (int)PIVOT_CONSTANTS.intakePos;
    private boolean isHardwareInitialized = false;


    @Override
    public void runOpMode() {

        if(!isHardwareInitialized) {
            initHardware(); // Init hardware can be called from outside this opmode
        }

        pivotController = new PIDController(PIVOT_CONSTANTS.kp, PIVOT_CONSTANTS.ki, PIVOT_CONSTANTS.kd);
        pivotController.setTolerance(10);
        double pivotPIDSpeedMultiplier = 1.0;

        // VARIABLES/OBJECTS (miscellaneous) ----------------------------------------------------------

        // drivetrain
        int driveSpeedIndex = 1; // change this to the index of the speed you want to start at
        double[] driveSpeedRange = {0.3, 0.64, 1};
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
        setupTimer = new ElapsedTime();
        pidTimer = new ElapsedTime();
        slideTimer = new ElapsedTime();

        // STATE MACHINE LOGIC

        boolean isIntakeProtected = false;
        boolean isGrabberOpen = true;
        boolean isArmPositionSet = true;

        // ACTUATOR LOGIC
        boolean isActuatorInitialized = false;

        // WAIT LOOP ----------------------------------------------------------------------------

        lightTimer.startTime();
        setupTimer.startTime();
        pidTimer.startTime();
        slideTimer.startTime();


        while (opModeInInit()) {

            // change drive mode
            if (gamepad1.right_bumper) {
                tankDrive = true;
            } else if (gamepad1.left_bumper) {
                tankDrive = false;
            }

            // START
            telemetry.addLine("TELEOP VERSION 2.2.2 + 2.5.0 AUTO-EDIT RELEASE");
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
        setupTimer.reset();
        pidTimer.reset();
        slideTimer.reset();
        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;

        double desiredArmTheta = 0; 
        double desiredArmLength = 0; 

        // RUN LOOP -----------------------------------------------------------------------------

        while (opModeIsActive()) {
            linearActuatorRight.setPower(0);
            linearActuatorLeft.setPower(0);

            currentSampleDistance = sampleSensor.getDistance(DistanceUnit.CM);

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

                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);


                        isRunningPivotToPosition = false;
                        linearPivotRight.setPower(0);
                        linearPivotLeft.setPower(0);

                        lightTimer.reset();
                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }

                    // ATTEMPT/DIVIDE SAMPLE

                    // attempt to grab a sample (if safe)
                    if (gamepad2.right_trigger > 0.1 && !checkGTwoRT &&
                        linearSlideAvgPosition > 100) {
                            checkGTwoRT = true;
                            linearSlideRight.setPower(0);
                            linearSlideLeft.setPower(0);
                            isStateInitialized = false;
                            linearSlideState = LinearSlideStates.INTAKE_ATTEMPT_SAMPLE;
                    } else if (gamepad2.right_bumper && !checkGTwoRB &&
                        linearSlideAvgPosition > 100) {
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

                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);

                        linearPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        linearPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_RETRACT;
                    }

                    // start sequence to pivot to specimen
                    if (gamepad2.left_bumper && !checkGTwoLB) {
                        checkGTwoLB = true;
                        specimanning = true;

                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);

                        linearPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        linearPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_RETRACT;
                    }
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

                    // SERVOS
                    // finish claw machine grab after a set amount of time
                    if (lightTimer.seconds() > 3.0 || isPossessingSample(currentSampleDistance)) {

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
                    } else if (lightTimer.seconds() > 3.0) {
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

                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);

                        linearPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        linearPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                        isStateInitialized = false;
                        isIntakeProtected = false;
                        linearSlideState = LinearSlideStates.INTAKE_RETRACT;
                    }

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

                    if (linearSlideAvgPosition >= SLIDE_CONSTANTS.depositEndRetract && !isLinearSlideFullyRetracted(limitSwitch)) {
                        double timeAccel = 1;
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
                    if (linearSlideAvgPosition < SLIDE_CONSTANTS.intakeEndRetract || isLinearSlideFullyRetracted(limitSwitch)) {
                        isStateInitialized = false;
                        if (specimanning) {
                            linearSlideState = LinearSlideStates.PIVOT_TO_SPECIMEN_GRAB;
                        } else {
                            // normal mode
                            linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT;
                        }

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
                    break;

            // move lift from intake mode to deposit mode
                case PIVOT_TO_DEPOSIT:

                    if (!isStateInitialized) {
                        linearPivotTargetPosition = (int)PIVOT_CONSTANTS.depositPos;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();

                        lightTimer.reset();
                        isIntakeProtected = true;
                        isArmPositionSet = true;
                        isStateInitialized = true;
                    }


                    // stop slides once finished retracting
                    // (slides started retracting in INTAKE_RETRACT)
                    // if (isLinearSlideFullyRetracted(limitSwitch)) {
                    //     linearSlideRight.setPower(0);
                    //     linearSlideLeft.setPower(0);
                    //     linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //     linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //     linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //     linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //     isArmPositionSet = true;
                    // } else if (!isArmPositionSet) {
                    //     linearSlideRight.setPower(-SLIDE_SPEED);
                    //     linearSlideLeft.setPower(-SLIDE_SPEED);
                    // }


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
                    // for deposit/intake
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
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
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_RETRACT_SET;
                    }

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
                    break;

            // retract slide so it can pivot
                case DEPOSIT_RETRACT:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        lightTimer.reset();
                        isStateInitialized = true;
                    }

                    if (linearSlideAvgPosition >= SLIDE_CONSTANTS.depositEndRetract && !isLinearSlideFullyRetracted(limitSwitch)) {
                        double timeAccel = 1;
                        linearSlideRight.setPower((-timeAccel)+SLIDE_CONSTANTS.gravityCoefficient);
                        linearSlideLeft.setPower((-timeAccel)+SLIDE_CONSTANTS.gravityCoefficient);
                    } else {
                        linearSlideRight.setPower(0);
                        linearSlideLeft.setPower(0);
                    }


                    // exit mode if slide has gone far enough to safely start pivoting
                    if (linearSlideAvgPosition < SLIDE_CONSTANTS.depositEndRetract || isLinearSlideFullyRetracted(limitSwitch)) {
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                    }
                    break;

            // move lift from deposit mode to intake mode
                case PIVOT_TO_INTAKE:

                    if (!isStateInitialized) {
                        linearPivotTargetPosition = (int)PIVOT_CONSTANTS.intakePos;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();


                        lightTimer.reset();
                        isArmPositionSet = true;
                        isStateInitialized = true;
                    }

                    linearSlideLeft.setPower(0);
                    linearSlideRight.setPower(0);


                    // stop slides once finished retracting
                    // (slides started retracting in DEPOSIT_RETRACT)
                    // if (isLinearSlideFullyRetracted(limitSwitch)) {
                    //     linearSlideRight.setPower(0);
                    //     linearSlideLeft.setPower(0);
                    //     linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //     linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //     linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //     linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //     isArmPositionSet = true;
                    // } else if (!isArmPositionSet){
                    //     linearSlideRight.setPower(-SLIDE_SPEED);
                    //     linearSlideLeft.setPower(-SLIDE_SPEED);
                    // }

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
                        duckSpinner.setPower(DUCK_VALUES.spinStop);

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
                        specimenGrabber.setPosition(SERVO_VALUES.specimenGrabberClosePos);
                        isGrabberOpen = false;
                        isStateInitialized = false;
//                        linearSlideState = LinearSlideStates.SPECIMEN_POSITION;
                        linearSlideState = LinearSlideStates.SPECIMEN_PLACE;
                    }

                    // go back to intake mode
                    if (gamepad2.left_bumper && !checkGTwoLB) {
                        checkGTwoLB = true;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                    }
                    break;


            // allows robot to position for a specimen hang
                case SPECIMEN_POSITION:
                    if (!isStateInitialized) {
                        duckSpinner.setPower(DUCK_VALUES.spinRest);

                        linearPivotTargetPosition = (int)PIVOT_CONSTANTS.specimenPositionPos;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();

                        intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);

                        isStateInitialized = true;
                    }

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
                        duckSpinner.setPower(DUCK_VALUES.spinHyperActive);

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
                    /*if (gamepad2.right_trigger > 0.1 && !checkGTwoRT) {
                        checkGTwoRT = true;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.SPECIMEN_POSITION;
                    }*/
                    break;


            // go back from placing to grabbing specimens
                case SPECIMEN_RETRACT:
                    if (!isStateInitialized) {
                        duckSpinner.setPower(DUCK_VALUES.spinRest);

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
                        double timeAccel = 1;
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
                    if (gamepad2.dpad_up) {
                        linearActuatorRight.setPower(ACTUATOR_SPEED);
                        linearActuatorLeft.setPower(ACTUATOR_SPEED);
                    } else if (gamepad2.dpad_down) {
                        linearActuatorRight.setPower(-ACTUATOR_SPEED);
                        linearActuatorLeft.setPower(-ACTUATOR_SPEED);
                    } else {
                        linearActuatorRight.setPower(0);
                        linearActuatorLeft.setPower(0);
                    }


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
            // activated by driver 1
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

            
            } // end lift state machine

            // Preventing slide down of gravity if able 
            if(fightGravity && linearSlideState == LinearSlideStates.DEPOSIT_ACTIVE) {
                linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlideLeft.setPower(SLIDE_CONSTANTS.gravityCoefficient);
                linearSlideRight.setPower(SLIDE_CONSTANTS.gravityCoefficient);
            }

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

// LINEAR ACTUATORS SETUP ------------------------------------------------------------------------

            // get the hands to the relative correct position for hanging
            // there is a safety to lower in case things go wrong (default case)
            
            linearActuatorRight.setPower(0);
            linearActuatorLeft.setPower(0);
            
            if (gamepad2.dpad_left) {
                // lower actuators manually to reset in case of malfunction
                linearActuatorRight.setPower(-ACTUATOR_SPEED);
                linearActuatorLeft.setPower(-ACTUATOR_SPEED);
                isActuatorInitialized = true;
            } else if (!isActuatorInitialized && setupTimer.seconds() <= 5.0) {

                // apply power to move actuators up for a set amount of time
                linearActuatorRight.setPower(ACTUATOR_SPEED);
                linearActuatorLeft.setPower(ACTUATOR_SPEED);
            }

// TELEMETRY ------------------------------------------------------------------------------------
            if(!isTelemetrySuppresed) { 
                telemetry.addData("Duck Disabled", disableDuck);
                telemetry.addLine("MANUAL OVERRIDE: (gamepad 2) dpad_down + button_a");
                telemetry.addLine("-------------------------");

                telemetry.addData("SLIDE STATE", linearSlideState);
                telemetry.addData("TIME", lightTimer.seconds());
                telemetry.addLine("-------------------------");

                telemetry.addLine("ACTUATORS");
                telemetry.addData("LAR POW", linearActuatorRight.getPower());
                telemetry.addData("LAL POW", linearActuatorLeft.getPower());
                telemetry.addLine("-------------------------");

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
                telemetry.addLine("-------------------------");
            }


            if (camera) {
                if (glasses.hasAprilTag()) {
                    telemetry.addLine("VISION");
                    glasses.addAprilTagTelemetry();
                }
            }

        }
    } // end run opmode method


    /**
     * Used in automation. Determines possession based on distance.
     * @param currentSampleDistance current distance reading from the sample sensor
     * @return whether intake can see it has collected a sample
     **/
    public boolean isPossessingSample(double currentSampleDistance) {
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
    public boolean isLinearSlideFullyRetracted(boolean limitSwitch) {
        return limitSwitch || gamepad2.x || gamepad2.y;
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
        pivotPower = (-gamepad2.left_stick_y)*(PIVOT_SPEED*0.7)*pivotCushion;

        // apply a factor to fight gravity if needed (works and doesn't work at the same time)
        // note that this does not account for the added load due to lever action
        // if (pivotPosition > 100) {
        //     pivotPower += pivotFF;
        // }
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

    public double getLinearPivotAvgPosition() {
        return (linearPivotLeft.getCurrentPosition() + linearPivotRight.getCurrentPosition()) * 0.5;
    }

    
    public double getLinearSlideAvgPosition() {
        return (linearSlideLeft.getCurrentPosition() + linearSlideRight.getCurrentPosition()) * 0.5;
    }

    /**
     * Initializes all hardware needed to begin teleop
     */
    public void initHardware() {
        isHardwareInitialized = true;        

        // HARDWARE CONFIGURATION
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        linearSlideRight = hardwareMap.get(DcMotorEx.class, "linearSlideRight");
        linearSlideLeft = hardwareMap.get(DcMotorEx.class, "linearSlideLeft");

        linearPivotRight = hardwareMap.get(DcMotorEx.class, "linearPivotRight");
        linearPivotLeft = hardwareMap.get(DcMotorEx.class, "linearPivotLeft");

        linearActuatorRight = hardwareMap.get(CRServo.class, "linearActuatorRight");
        linearActuatorLeft = hardwareMap.get(CRServo.class, "linearActuatorLeft");

        intakeWheelR = hardwareMap.get(CRServo.class, "intakeWheelR");
        intakeWheelL = hardwareMap.get(CRServo.class, "intakeWheelL");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        duckSpinner = hardwareMap.get(CRServo.class, "duckSpinner");

        specimenGrabber = hardwareMap.get(Servo.class, "specimenGrabber");

        sampleSensor = hardwareMap.get(ColorRangeSensor.class, "sampleSensor");
        linearSlideSwitch = hardwareMap.get(TouchSensor.class, "linearSlideSwitch");

        imu = hardwareMap.get(IMU.class, "imu");
        heightSensor = hardwareMap.get(DistanceSensor.class, "heightSensor");

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

    public void setFightingGravity(boolean bool) {
        this.fightGravity = bool;
    }
}
