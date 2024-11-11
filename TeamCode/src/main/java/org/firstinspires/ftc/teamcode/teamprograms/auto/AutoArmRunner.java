package org.firstinspires.ftc.teamcode.teamprograms.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Welcome!
 * Teleop Version: A0.6.0 BETA (AUTO-EDIT)
 * STARTING POSITION/STATE: INTAKE_ACTIVE
 **/

 /*--------------------------
 * CONTROLS
 * --------------------------
 * Gamepad 1 (drivetrain)
 * [right_stick_y] - Right drive wheels
 * [left_stick_y] - Left drive wheels
 * [right_trigger] - Slide right
 * [left_trigger] - Slide left
 * [right_bumper] - Increase max speed
 * [left_bumper] - Lower max speed
 * [dpad_right + x_button] - reset pivot if disconnected (DANGEROUS)
 * --------------------------
 * Gamepad 2 (lift)
 * [right_stick_y] - extend/retract slides
 * [left_stick_y] - pivot slides (only in HANG_TIME)
 * [right_bumper] - divide samples (intake)
 * [right_bumper] - set intake wheels to empty (deposit)
 * [left_bumper] - move to old deposit position (intake)
 * [left_bumper] - set intake wheels to low speed (deposit)
 * [b_button] - reverse direction of intake wheels (intake)
 * [left_trigger] - flip between intake and deposit
 * [right_trigger] - move intake to and from deposit position (deposit)
 * [right_trigger] - attempt sample grab (intake)
 * [y_button] - empty intake when full (intake full)
 * [x_button/y_button] - override limit switch (retract and pivot states)
 * [dpad_down + a_button] - enter MANUAL_OVERRIDE
 * --------------------------
 * Combined gamepad controls
 * [gamepad1 dpad_up + gamepad2 dpad_up] - enter/exit HANG_TIME (only in DEPOSIT_ACTIVE)
 * --------------------------
 * MANUAL_OVERRIDE (affects Gamepad 2 only)
 * NOTE: this mode is EXTREMELY DANGEROUS (yes, more than last year)
 *      BE ALERT: the worm gear does not have an automatic stop;
 *      if you are not careful, the robot can be bent severely
 * --------------------------
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

@Config
// @Disabled
@TeleOp(name="AutoArmRunner Test Suite", group="teamprograms")
public class AutoArmRunner extends LinearOpMode {
    protected boolean isTelemetrySuppresed = false;

    // HARDWARE
    protected DcMotorEx frontRight, backRight, frontLeft, backLeft;
    protected DcMotorEx linearSlideLift, linearSlidePivot;
    protected CRServo intakeWheelR, intakeWheelL;
    protected Servo intakePivot;

    protected ColorRangeSensor sampleSensor;
    protected TouchSensor linearSlideSwitch, pivotResetSwitch;

    // SERVO POSITION VALUES (editable by FTC dashboard)
    public static class ServoValues {
        public double pivotIntakePos = 0.44;
        public double pivotDepositPos = 0.85;
        public double pivotAlternateDepositPos = 0.34;
        public double pivotRestPos = 0.52;
        public double pivotHangPos = 0.2;
    }
    public static ServoValues SERVO_VALUES = new ServoValues();

    // more servo variables
    final double INTAKE_POWER_MAX = 1.0;
    final double INTAKE_POWER_HOLD = 0.06;
    final double INTAKE_POWER_EMPTY = -0.3;
    final double INTAKE_POWER_ZERO = 0;


    // SENSOR VARIABLES (editable by FTC dashboard)
    public static class SensorVariables {
        public float sampleSensorGain = 1.0f;
        public double sampleDistance = 3.2;
        public double sampleCodeBlue = 0.007;
    }
    public static SensorVariables SENSOR_VARIABLES = new SensorVariables();

    // SLIDE VARIABLES
    public static class SlideConstants {
        public double gravityCoefficient = 0.0005;
        public double extensionLimit = 2400;
        public double topBucketHeight = 3800;
        public double bottomBucketHeight = 1900;
        public double cushionRatio = 400;
        public double topBucketHeightAlternate = 4200;
        public double bottomBucketHeightAlternate = 2200;
        public double depositEndRetract = 1200;
        public double intakeEndRetract = 1200;
    }
    public static SlideConstants SLIDE_CONSTANTS = new SlideConstants();

    public static class PivotConstants {
        public double cushionRatio = 400;
        public int resetLevelPos = 100;
        public int hangPos = 3700;
        public int attmeptSamplePos = 100;
    }
    public static PivotConstants PIVOT_CONSTANTS = new PivotConstants();

    final int PIVOT_MAX_POSITION = 3600;
    final int PIVOT_INTAKE_RETRACT_POSITION = 400;
    final int PIVOT_MIN_POSITION = 0;
    final int PIVOT_ALTERNATE_DEPOSIT_POSITION = 2850;
    final int PIVOT_ALTERNATE_RETRACT_SET_POSITION = 3200;
    final int PIVOT_CHAMBER = 2300;



    // ROBOT LIFT STATES
    enum LinearSlideStates {

        INTAKE_ACTIVE, INTAKE_ATTEMPT_SAMPLE, INTAKE_DIVIDE_SAMPLE, INTAKE_FULL, INTAKE_EMPTY,
        INTAKE_RETRACT_SET, INTAKE_RETRACT, INTAKE_EXTEND, PIVOT_TO_DEPOSIT, PIVOT_TO_CHAMBER,

        DEPOSIT_ACTIVE, DEPOSIT_RETRACT_SET, DEPOSIT_RETRACT, PIVOT_TO_INTAKE,
        DEPOSIT_ALTERNATE_ACTIVE, DEPOSIT_ALTERNATE_RETRACT_SET, PIVOT_REVERSE_ALTERNATE,
        HANG_TIME, PIVOT_TO_HANG_TIME,

        MANUAL_OVERRIDE, MANUAL_LOWER_PIVOT, MANUAL_RAISE_PIVOT, MANUAL_PIVOT_STASIS, // not functional?
        RESET_PIVOT_LOWER, RESET_PIVOT_LEVEL
    }

    LinearSlideStates linearSlideState;

    ElapsedTime lightTimer;


    // constant variables
    final double SLIDE_SPEED = 0.7; // was 0.5
    final double PIVOT_SPEED = 1.0;
    final int PIVOT_TOLERANCE = 33; // How far the pivot can be from its position before stopping
    private double pivotReverseFactor = 1; // Set AutoInit.driveMotorToIterative

    boolean tankDrive = true;

    boolean alternateDeposit = true;

    int manualPivotCheckpoint = 0;


    @Override
    public void runOpMode() {

        initHardware();

        // VARIABLES/OBJECTS (miscellaneous) ----------------------------------------------------------

        // drivetrain
        int driveSpeedIndex = 1; // change this to the index of the speed you want to start at
        double[] driveSpeedRange = {0.3, 0.6, 1};
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

        boolean checkGOneDDOWN = true;
        boolean checkGOneDUP = true;
        boolean checkGOneA = true;

        boolean checkGOneDRIGHT = true;
        boolean checkGOneX = true;
        boolean checkGOneY = true;

        lightTimer = new ElapsedTime();

        // STATE MACHINE LOGIC

        boolean isStateInitialized = false;
        boolean isIntakeProtected = false;
        boolean isArmPositionSet = true;
//        boolean overrideSampleSensor = true;

        // WAIT LOOP ----------------------------------------------------------------------------

        lightTimer.startTime();


        while (opModeInInit()) {

            // change drive mode
            if (gamepad1.a) {
                tankDrive = true;
            } else if (gamepad1.b) {
                tankDrive = false;
            }

            // START
            telemetry.addLine("TELEOP VERSION A0.6.0 BETA (AUTO EDIT)");
            telemetry.addLine("-------------------------");
//            telemetry.addData("COLOR SENSOR OVERRIDDEN", overrideSampleSensor);
            telemetry.addData("TANK DRIVE", tankDrive);
            telemetry.addLine("Press Start");
            telemetry.addLine("-------------------------");
            telemetry.addLine("CONTROLLER 1  BUTTON A: TANK DRIVE");
            telemetry.addLine("CONTROLLER 1 BUTTON B: MECANUM DRIVE");
            telemetry.update();
        }

        waitForStart();

        lightTimer.reset();
        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;



        // RUN LOOP -----------------------------------------------------------------------------

        while (opModeIsActive()) {


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

            if (!gamepad1.dpad_down) checkGOneDDOWN = false;
            if (!gamepad1.dpad_up) checkGOneDUP = false;
            if (!gamepad1.a) checkGOneA = false;

            if (!gamepad1.dpad_right) checkGOneDRIGHT = false;
            if (!gamepad1.x) checkGOneX = false;
            if (!gamepad1.y) checkGOneY = false;


            // LIFT ----------------------------------------------------------------------------

            // gather values
            double linearSlidePower = 0;
            double linearSlidePosition = linearSlideLift.getCurrentPosition();
            boolean limitSwitch = linearSlideSwitch.isPressed();

            double currentSampleDistance = sampleSensor.getDistance(DistanceUnit.CM);

            double pivotPosition = linearSlidePivot.getCurrentPosition();
            boolean resetSwitch = pivotResetSwitch.isPressed();


            // enter MANUAL_OVERRIDE
            if (gamepad2.dpad_down && !checkGTwoDDOWN && gamepad2.a && !checkGTwoA &&
                linearSlideState != LinearSlideStates.MANUAL_OVERRIDE) {
                checkGTwoDDOWN = true;
                checkGTwoA = true;
                intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                linearSlideState = LinearSlideStates.MANUAL_OVERRIDE;
            }

            // enter PIVOT_RESET states
            // NOTE: this is dangerous
            // if (gamepad1.dpad_right && !checkGOneDRIGHT &&
            //     gamepad1.x && !checkGOneX &&
            //     linearSlideState != LinearSlideStates.RESET_PIVOT_LOWER &&
            //     linearSlideState != LinearSlideStates.RESET_PIVOT_LEVEL) {
            //     checkGOneDRIGHT = true;
            //     checkGOneX = true;
            //     isStateInitialized = false;
            //     linearSlideState = LinearSlideStates.RESET_PIVOT_LOWER;
            // }

            // DANGEROUS ADJUSTMENTS

           // lower pivot slightly and reset position to 0
            // (if robot disconnected)
            if (gamepad1.dpad_down && !checkGOneDDOWN &&
                linearSlideState != LinearSlideStates.MANUAL_LOWER_PIVOT) {
                checkGOneDDOWN = true;
                linearSlideState = LinearSlideStates.MANUAL_LOWER_PIVOT;
            }

            // raise pivot slightly and reset position to 0
            // (if robot disconnected)
            if (gamepad1.dpad_up && !checkGOneDUP &&
                linearSlideState != LinearSlideStates.MANUAL_RAISE_PIVOT) {
                checkGOneDUP = true;
                linearSlideState = LinearSlideStates.MANUAL_RAISE_PIVOT;
            }

            // exit manual pivot adjustment when finished
            if (gamepad1.a && !checkGOneA &&
            linearSlideState == LinearSlideStates.MANUAL_PIVOT_STASIS) {
                checkGOneA = true;
                linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
            }

// LIFT STATE MACHINE ----------------------------------------------------------------------------

            switch (linearSlideState) {

            // lift in position to grab samples
            // speed of intake wheels is at max
                case INTAKE_ACTIVE:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);

                        linearSlidePivot.setTargetPosition(PIVOT_INTAKE_RETRACT_POSITION);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(PIVOT_SPEED);
                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }

                    // PIVOT

                    if (Math.abs(pivotPosition-PIVOT_INTAKE_RETRACT_POSITION) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    }

                    // SLIDES

                    linearSlidePower = calculateSlidePower(linearSlidePosition, true);
                    linearSlideLift.setPower(linearSlidePower);

                    // ATTEMPT/DIVIDE SAMPLE

                    // attempt to grab a sample (if safe)
                    if (gamepad2.right_trigger > 0.1 && !checkGTwoRT &&
                        linearSlidePosition > 500) {
                        checkGTwoRT = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ATTEMPT_SAMPLE;
                    } else if (gamepad2.right_bumper && !checkGTwoRB &&
                        linearSlidePosition > 500) {
                        checkGTwoRB = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_DIVIDE_SAMPLE;
                    }



                    // EXIT INTAKE

                    // start sequence to pivot to deposit (ALTERNATE)
                    if(isArmPositionSet && gamepad2.dpad_left && gamepad2.left_trigger > 0.1) {
                        checkGTwoLT = true;
                        isStateInitialized = false;
                        linearSlidePivot.setPower(0);
                        linearSlideState = LinearSlideStates.PIVOT_TO_CHAMBER;
                    } else if (isArmPositionSet && gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        alternateDeposit = true;
                        linearSlideState = LinearSlideStates.INTAKE_RETRACT_SET;

                    } else if (isArmPositionSet && gamepad2.left_bumper && !checkGTwoLB) {
                        // OLD
                        checkGTwoLB = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        alternateDeposit = false;
                        linearSlideState = LinearSlideStates.INTAKE_RETRACT_SET;
                    }
                    break;

            // robot attempts to grab a sample
            // intake cannot be messed with while in this mode
                case PIVOT_TO_CHAMBER:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);

                        // Driving the pivot the the target with the given power and tolerance
                        linearSlidePivot.setTargetPosition(PIVOT_CHAMBER); // Used only to interface with older code
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isIntakeProtected = true;
                        // isArmPositionSet = false;
                        isArmPositionSet = true;
                        isStateInitialized = true;
                    }

                    // linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linearSlidePower = calculateSlidePower(linearSlidePosition, false);
                    linearSlideLift.setPower(linearSlidePower);

                    // EXIT

                    // make deposit accessible once lift has finished pivoting
                    // (and once slide has finished retracting)
                    if (Math.abs(pivotPosition-linearSlidePivot.getTargetPosition()) < PIVOT_TOLERANCE &&
                        isArmPositionSet) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        pivotReverseFactor = 1; // Reset the factor 
                        linearSlideState = LinearSlideStates.DEPOSIT_ALTERNATE_ACTIVE;
                    } else {
                        pivotReverseFactor *= AutoInit.driveMotorToIterative(
                            linearSlidePivot, 
                            linearSlidePivot.getTargetPosition(), 
                            PIVOT_TOLERANCE, 
                            pivotReverseFactor * PIVOT_SPEED
                        );
                    }

                    // ABORT

                    // go back to intake if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                    }
                    break;

            // lift in position to score samples (OLD)
                case INTAKE_ATTEMPT_SAMPLE:

                    if (!isStateInitialized) {
                        intakeWheelR.setPower(INTAKE_POWER_MAX);
                        intakeWheelL.setPower(INTAKE_POWER_MAX);
                        intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);
                        linearSlidePivot.setTargetPosition(PIVOT_CONSTANTS.attmeptSamplePos);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(-PIVOT_SPEED);
                        isArmPositionSet = false;
                        lightTimer.reset();
                        isStateInitialized = true;
                    }

                    // PIVOT

                    // move pivot down to 0 so it can grab a sample
                    if (Math.abs(pivotPosition-PIVOT_CONSTANTS.attmeptSamplePos) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    }

                    // SLIDES
                    linearSlidePower = calculateSlidePower(linearSlidePosition, true);
                    linearSlideLift.setPower(linearSlidePower);

                    // SERVOS
                    // finish claw machine grab after a set amount of time
                    if (isArmPositionSet && lightTimer.seconds() > 0.8) {

                        // robot has successfully acquired a sample
                        if (isPossessingSample(currentSampleDistance)) {
                            linearSlideLift.setPower(0);
                            isStateInitialized = false;
                            linearSlideState = LinearSlideStates.INTAKE_FULL;
                        } else {
                            // robot did not get sample
                            linearSlideLift.setPower(0);
                            isStateInitialized = false;
                            linearSlideState = LinearSlideStates.INTAKE_EMPTY;
                        }
                    } else if (isArmPositionSet && lightTimer.seconds() > 0.5) {
                        // bring intake up for a short period to secure sample
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                    }

                    // ABORT ATTEMPT

                    // leave sample attempt early if mistaken
                    if (gamepad2.right_trigger > 0.1 && !checkGTwoRT) {
                        checkGTwoRT = true;
                        linearSlideLift.setPower(0);
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_EMPTY;

                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    break;

            // attempt to split the samples close together for easier grabbing
                case INTAKE_DIVIDE_SAMPLE:

                    if (!isStateInitialized) {
                        intakeWheelR.setPower(INTAKE_POWER_EMPTY);
                        intakeWheelL.setPower(INTAKE_POWER_EMPTY);
                        intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);
                        linearSlidePivot.setTargetPosition(PIVOT_MIN_POSITION);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(-PIVOT_SPEED);
                        isArmPositionSet = false;
                        lightTimer.reset();
                        isStateInitialized = true;
                    }

                    // PIVOT

                    // move pivot down to 0 so it can grab a sample
                    if (Math.abs(pivotPosition-PIVOT_MIN_POSITION) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    }

                    // SLIDES
                    linearSlidePower = calculateSlidePower(linearSlidePosition, true);
                    linearSlideLift.setPower(linearSlidePower);

                    // SERVOS
                    // finish claw machine grab after a set amount of time
                    if (isArmPositionSet && lightTimer.seconds() > 1) {
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
                    } else if (isArmPositionSet && lightTimer.seconds() > 0.8) {
                        // bring intake up for a short period to secure sample
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                    }

                    // ABORT DIVIDE

                    // leave sample divide early if mistaken
                    if (gamepad2.right_bumper && !checkGTwoRB) {
                        checkGTwoRB = true;
                        linearSlideLift.setPower(0);
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;

                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    break;

            // robot is in possession of a sample
            // speed of intake wheels changes to accommodate
                case INTAKE_FULL:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isIntakeProtected = true;
                        intakeWheelR.setPower(INTAKE_POWER_HOLD);
                        intakeWheelL.setPower(INTAKE_POWER_HOLD);

                        linearSlidePivot.setTargetPosition(PIVOT_INTAKE_RETRACT_POSITION);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(PIVOT_SPEED);
                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }

                    // PIVOT
                    if (Math.abs(pivotPosition-PIVOT_INTAKE_RETRACT_POSITION) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    }


                    // SLIDES
                    linearSlidePower = calculateSlidePower(linearSlidePosition, true);
                    linearSlideLift.setPower(linearSlidePower);

                    // SERVOS
                    // move intake pivot to and from protected/default position
                    if (gamepad2.right_trigger > 0.1 && !checkGTwoRT && !isIntakeProtected) {
                        checkGTwoRT = true;
                        intakePivot.setPosition(SERVO_VALUES.pivotDepositPos);
                        isIntakeProtected = true;
                    } else if (gamepad2.right_trigger > 0.1 && !checkGTwoRT && isIntakeProtected) {
                        checkGTwoRT = true;
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isIntakeProtected = false;
                    }

                    // TODO: remove once color recognition implemented
                    if (gamepad2.y) {
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        isIntakeProtected = false;
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        linearSlideState = LinearSlideStates.INTAKE_EMPTY;
                    }

                    // EXIT INTAKE

                    // start sequence to pivot to deposit (ALTERNATE)
                    if (isArmPositionSet && gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        isIntakeProtected = false;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        alternateDeposit = true;
                        linearSlideState = LinearSlideStates.INTAKE_RETRACT_SET;

                    } else if (isArmPositionSet && gamepad2.left_bumper && !checkGTwoLB) {
                        // OLD
                        checkGTwoLB = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        isIntakeProtected = false;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        alternateDeposit = false;
                        linearSlideState = LinearSlideStates.INTAKE_RETRACT_SET;
                    }
                    break;

            // spit out sample collected (if it did)
                case INTAKE_EMPTY:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotDepositPos);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);
                        lightTimer.reset();
                        isStateInitialized = true;
                    }

                    linearSlidePower = calculateSlidePower(linearSlidePosition, true);
                    linearSlideLift.setPower(linearSlidePower);

                    if (lightTimer.seconds() > 0.5) {
                        // exit emptying
                        linearSlideLift.setPower(0);
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

            // ready lift servos to retract slide
                case INTAKE_RETRACT_SET:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        linearSlidePivot.setTargetPosition(PIVOT_INTAKE_RETRACT_POSITION);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(PIVOT_SPEED);
                        lightTimer.reset();
                        isStateInitialized = true;
                    }



                    // EXIT

                    // stop pivot when it has reached the set position
                    // go to retract slides
                    if (Math.abs(pivotPosition-PIVOT_INTAKE_RETRACT_POSITION) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_RETRACT;
                    }


                    // ABORT

                    // go back to intake if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                    }
                    break;

            // retract slide so it can pivot
                case INTAKE_RETRACT:

                    if (!isStateInitialized) {
                        isStateInitialized = true;
                    }

                    if (!limitSwitch && linearSlidePosition > SLIDE_CONSTANTS.intakeEndRetract) {
                        linearSlideLift.setPower(-1.0);
                    } else {
                        linearSlideLift.setPower(0);
                    }

                    // EXIT

                    // start exiting right before slide hits 0
                    // (attempts to make transition faster and smoother)
                    if (
                        linearSlidePosition <= SLIDE_CONSTANTS.intakeEndRetract 
                        || isLinearSlideFullyRetracted(limitSwitch)
                    ) {
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT;
                    }


                    // ABORT

                    // go back to intake if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                    }
                    break;

            // move lift from intake mode to deposit mode
            // works for both (OLD) (ALTERNATE)
                case PIVOT_TO_DEPOSIT:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        // make pivot go to correct position based on deposit selection
                        // if (alternateDeposit) {
                        //     linearSlidePivot.setTargetPosition(PIVOT_ALTERNATE_DEPOSIT_POSITION);
                        // } else {
                        //     linearSlidePivot.setTargetPosition(PIVOT_MAX_POSITION);
                        // }
                        // linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        // linearSlidePivot.setPower(PIVOT_SPEED);

                        // LARSON EDIT: The comment-out section is replaced by code that does similar without inital delay.
                        int target;
                        if (alternateDeposit) {
                            target = PIVOT_ALTERNATE_DEPOSIT_POSITION;
                        } else {
                            target = PIVOT_ALTERNATE_DEPOSIT_POSITION;
                            // target = PIVOT_DEPOSIT_POSITION;
                        }

                        
                        // Driving the pivot the the target with the given power and tolerance
                        linearSlidePivot.setTargetPosition(target); // Used only to interface with older code
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isIntakeProtected = true;
                        // isArmPositionSet = false;
                        isArmPositionSet = true;
                        isStateInitialized = true;
                    }

                    // linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linearSlidePower = calculateSlidePower(linearSlidePosition, false);
                    linearSlideLift.setPower(linearSlidePower);


                    // stop slides once finished retracting
                    // (slides started retracting in INTAKE_RETRACT)
                    // if (isLinearSlideFullyRetracted(limitSwitch)) {
                    //     linearSlideLift.setPower(0);
                    //     linearSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //     linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //     isArmPositionSet = true;
                    // } else if (!isArmPositionSet) {
                    //     linearSlideLift.setPower(-1.0);
                    // }


                    // EXIT

                    // make deposit accessible once lift has finished pivoting
                    // (and once slide has finished retracting)
                    if (Math.abs(pivotPosition-linearSlidePivot.getTargetPosition()) < PIVOT_TOLERANCE &&
                        isArmPositionSet) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        pivotReverseFactor = 1; // Reset the factor 

                        if (alternateDeposit) {
                            linearSlideState = LinearSlideStates.DEPOSIT_ALTERNATE_ACTIVE;
                        } else {
                            linearSlideState = LinearSlideStates.DEPOSIT_ACTIVE;
                        }
                    } else {
                        pivotReverseFactor *= AutoInit.driveMotorToIterative(
                            linearSlidePivot, 
                            linearSlidePivot.getTargetPosition(), 
                            PIVOT_TOLERANCE, 
                            pivotReverseFactor * PIVOT_SPEED
                        );
                    }

                    // ABORT

                    // go back to intake if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                    }
                    break;

            // lift in position to score samples (OLD)
                case DEPOSIT_ACTIVE:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isIntakeProtected = true;
                        intakeWheelR.setPower(INTAKE_POWER_HOLD);
                        intakeWheelL.setPower(INTAKE_POWER_HOLD);
                        isStateInitialized = true;
                    }

                    // SLIDES
                    linearSlidePower = calculateSlidePower(linearSlidePosition, false);
                    linearSlideLift.setPower(linearSlidePower);

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
                        linearSlideLift.setPower(SLIDE_CONSTANTS.gravityCoefficient);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_RETRACT_SET;
                    }

                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.HANG_TIME;
                    }
                    break;

            // ready lift servos to retract slide (OLD)
                case DEPOSIT_RETRACT_SET:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);
                        lightTimer.reset();
                        isStateInitialized = true;
                    }

                    // EXIT

                    // retract slides when pivot has been set
                    if (lightTimer.seconds() > 0) {
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_RETRACT;
                    }

                    // ABORT

                    // go back to deposit if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_ACTIVE;
                    }
                    break;

            // lift in position to score samples more accurately and safely (ALTERNATE)
            // NOTE: drivetrain must turn around in this mode to deposit
                case DEPOSIT_ALTERNATE_ACTIVE:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isIntakeProtected = true;
                        intakeWheelR.setPower(INTAKE_POWER_HOLD);
                        intakeWheelL.setPower(INTAKE_POWER_HOLD);
                        isStateInitialized = true;
                    }

                    // SLIDES
                    linearSlidePower = calculateSlidePower(linearSlidePosition, false);
                    linearSlideLift.setPower(linearSlidePower);

                    // SERVOS

                    // move intake pivot to and from deposit mode
                    if (gamepad2.right_trigger > 0.1 && !checkGTwoRT && isIntakeProtected) {
                        checkGTwoRT = true;
                        intakePivot.setPosition(SERVO_VALUES.pivotAlternateDepositPos);
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
                        // linearSlideState = LinearSlideStates.DEPOSIT_ALTERNATE_RETRACT_SET;
                        linearSlideState = LinearSlideStates.DEPOSIT_RETRACT;
                    }

                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_HANG_TIME;
                    }

                    break;

            // ready lift to retract safely (ALTERNATE)
                case DEPOSIT_ALTERNATE_RETRACT_SET:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);
                        linearSlidePivot.setTargetPosition(PIVOT_ALTERNATE_RETRACT_SET_POSITION);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(PIVOT_SPEED);
                        lightTimer.reset();
                        isStateInitialized = true;
                    }

                    // EXIT

                    // retract slides when pivot has been set
                    if (Math.abs(pivotPosition - PIVOT_ALTERNATE_RETRACT_SET_POSITION) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_RETRACT;
                    }

                    // ABORT

                    // go back to deposit if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_REVERSE_ALTERNATE;
                    }
                    break;

            // cancel pivot to intake while pivot is shifted and ready to retract (ALTERNATE)
            // also used for HANG_TIME transition
                case PIVOT_REVERSE_ALTERNATE:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        linearSlidePivot.setTargetPosition(PIVOT_ALTERNATE_DEPOSIT_POSITION);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(-PIVOT_SPEED);
                        isStateInitialized = true;
                    }

                    // EXIT

                    // go back to
                    if (Math.abs(pivotPosition - PIVOT_ALTERNATE_DEPOSIT_POSITION) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_ALTERNATE_ACTIVE;
                    }

                    // ABORT

                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_ALTERNATE_RETRACT_SET;
                    }

                    // for HANG_TIME
                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_TO_HANG_TIME;
                    }


                    break;

            // retract slide so it can pivot
                case DEPOSIT_RETRACT:

                    // TODO: eventually make slides slowly accelerate down

                    if (!limitSwitch && linearSlidePosition > SLIDE_CONSTANTS.depositEndRetract) {
                        linearSlideLift.setPower(-0.9);
                    } else {
                        linearSlideLift.setPower(0);
                    }


                    // exit mode if slide has gone far enough to safely start pivoting
                    // NOTE: The value here may not matter as there is a check again in PIVOT_TO_DEPOSIT
                    if (
                        linearSlidePosition <= SLIDE_CONSTANTS.depositEndRetract 
                        || isLinearSlideFullyRetracted(limitSwitch)
                    ) {
                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                    }

                    // ABORT

                    // go back to deposit if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        isIntakeProtected = true;
                        if (alternateDeposit) {
                            // alternate deposit pivots slightly upon retract set
                            // must go back to pivoting to deposit to fully reset
                            linearSlideState = LinearSlideStates.PIVOT_REVERSE_ALTERNATE;
                        } else {
                            linearSlideState = LinearSlideStates.DEPOSIT_ACTIVE;
                        }
                    }
                    break;

            // move lift from deposit mode to intake mode
                case PIVOT_TO_INTAKE:

                    if (!isStateInitialized) {
                        // linearSlidePivot.setTargetPosition(PIVOT_INTAKE_RETRACT_POSITION);
                        // linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        // linearSlidePivot.setPower(-PIVOT_SPEED);
                        // isArmPositionSet = false;
                        linearSlidePivot.setTargetPosition(PIVOT_INTAKE_RETRACT_POSITION); // Used only to interface with older code
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                        isStateInitialized = true;
                    }

                    // stop slides once finished retracting
                    // (slides started retracting in DEPOSIT_RETRACT)
                    if (isLinearSlideFullyRetracted(limitSwitch)) {
                        linearSlideLift.setPower(0);
                        linearSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    } else if (!isArmPositionSet){
                        linearSlideLift.setPower(-0.9);
                    }

                    linearSlidePower = calculateSlidePower(linearSlidePosition, false);
                    linearSlideLift.setPower(linearSlidePower);

                    // EXIT

                    // make intake accessible once lift has finished pivoting
                    // (and once slide has finished retracting)
                    if (Math.abs(pivotPosition-PIVOT_INTAKE_RETRACT_POSITION) < 10 &&
                        isArmPositionSet) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        pivotReverseFactor = 1; // Reset the direction and speed of pivot

                        // for in case deposit pivot was canceled
                        if (isPossessingSample(currentSampleDistance)) {
                            linearSlideState = LinearSlideStates.INTAKE_FULL;
                        } else {
                            linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
                        }
                    } else {
                        pivotReverseFactor *= AutoInit.driveMotorToIterative(
                            linearSlidePivot, 
                            PIVOT_INTAKE_RETRACT_POSITION, 
                            PIVOT_TOLERANCE, 
                            -pivotReverseFactor * PIVOT_SPEED
                        );
                    }

                    // ABORT

                    // go back to deposit if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        isIntakeProtected = true;
                        if (alternateDeposit && pivotPosition > PIVOT_ALTERNATE_DEPOSIT_POSITION) {
                            linearSlideLift.setPower(0);
                            linearSlideState = LinearSlideStates.PIVOT_REVERSE_ALTERNATE;
                        } else {
                            linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT;
                        }
                    }
                    
                    break;

            // allows slides to pivot manually (can enter from DEPOSIT_ACTIVE)
            // NOTE: this state is dangerous
                case HANG_TIME:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotHangPos);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);
                        isStateInitialized = true;
                    }

                    // TODO: make hang time slide power more sophisticated (find right speed ratios)

                    // determine cushion for the slide
                    double linearSlideCushion = (SLIDE_CONSTANTS.topBucketHeight - linearSlidePosition)
                            /SLIDE_CONSTANTS.cushionRatio;

                    // use the power cushion if robot is extending, ignore if retracting
                    if ((-gamepad2.right_stick_y) > 0) {
                        linearSlidePower = (((-gamepad2.right_stick_y)*SLIDE_SPEED*0.75)*
                                linearSlideCushion);
                    } else {
                        // make lowering slides slower
                        linearSlidePower = ((-gamepad2.right_stick_y)*SLIDE_SPEED*0.5);
                    }

                    // add coefficient to prevent slide from sliding down
                    linearSlidePower+=SLIDE_CONSTANTS.gravityCoefficient;

                    linearSlideLift.setPower(linearSlidePower);

//                    linearSlidePivot.setPower(calculateManualPivotPower(pivotPosition));

                    // EXIT

                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        linearSlideLift.setPower(0);
                        linearSlidePivot.setPower(0);
                        isStateInitialized = false;
                        if (alternateDeposit) {
                            // for alternate deposit, pivot must go backwards to deposit again
                            linearSlideState = LinearSlideStates.PIVOT_REVERSE_ALTERNATE;
                        } else {
                            linearSlideState = LinearSlideStates.DEPOSIT_ACTIVE;
                        }
                    }
                    break;

            // pivot slide to be able to hang (ALTERNATE)
                case PIVOT_TO_HANG_TIME:

                    if (!isStateInitialized) {
                        intakeWheelR.setPower(0);
                        intakeWheelL.setPower(0);
                        linearSlidePivot.setTargetPosition(PIVOT_CONSTANTS.hangPos);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(PIVOT_SPEED);
                        isStateInitialized = true;
                    }

                    // EXIT

                    // enable hanging when pivot position reached
                    if (Math.abs(pivotPosition - PIVOT_CONSTANTS.hangPos) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.HANG_TIME;
                    }


                    // ABORT

                    if (gamepad2.dpad_up && !checkGTwoDUP) {
                        checkGTwoDUP = true;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.PIVOT_REVERSE_ALTERNATE;
                    }

                    break;

            // in case everything else fails
            // NOTE: this state is dangerous
            // ALSO NOTE: this state is outdated
                case MANUAL_OVERRIDE:

                    linearSlidePower = 0;

                    // fight gravity
                    if (gamepad2.dpad_up) {
                        linearSlidePower +=SLIDE_CONSTANTS.gravityCoefficient;
                    }

                    // set power to lift motors
                    linearSlidePower +=(-gamepad2.right_stick_y*SLIDE_SPEED);
                    linearSlidePivot.setPower(calculateManualPivotPower(pivotPosition));
                    linearSlideLift.setPower(linearSlidePower);


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
                        // pivot to deposit position
                        intakePivot.setPosition(SERVO_VALUES.pivotDepositPos);
                    }

                    // reset the position of the linear slide
                    if (gamepad2.x) {
                        linearSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }


                    // exit MANUAL_OVERRIDE
                    if (gamepad2.dpad_down && !checkGTwoDDOWN && gamepad2.a && !checkGTwoA) {
                        checkGTwoDDOWN = true;
                        checkGTwoA = true;
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_RETRACT_SET;
                    }

                    break;

            // only driver 1 can enter this mode
            // NOTE: used for DISCONNECTS ONLY, mode can be dangerous
                case RESET_PIVOT_LOWER:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        linearSlidePivot.setPower(-0.3);
                        isStateInitialized = true;
                    }

                    // fully retract slides
                    if (!limitSwitch) {
                        linearSlideLift.setPower(-0.8);
                    } else {
                        linearSlideLift.setPower(0);
                    }


                    // EXIT

                    // leave once pivot hits the switch
                    // reset encoder position so pivot knows where it is and go to level mode
                    if (resetSwitch) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.RESET_PIVOT_LEVEL;
                    }
                    break;

            // move pivot back to 0 once reset switch has been hit
            // (used in case robot disconnects)
                case RESET_PIVOT_LEVEL:

                    if (!isStateInitialized) {
                        linearSlidePivot.setTargetPosition(PIVOT_CONSTANTS.resetLevelPos);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(PIVOT_SPEED);
                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }

                    // in case slide did not have time to fully retract
                    if (isLinearSlideFullyRetracted(limitSwitch)) {
                        linearSlideLift.setPower(0);
                        isArmPositionSet = true;
                    }


                    // EXIT

                    // go back to INTAKE_ACTIVE (normal functions) once arm has finished resetting
                    if (Math.abs(pivotPosition - PIVOT_CONSTANTS.resetLevelPos) < 10 &&
                        isArmPositionSet) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
                    }
                    break;

            // lower pivot slightly and reset position to 0
            // (if robot disconnected)
                // TODO: remove once reset switch works
                case MANUAL_LOWER_PIVOT:
                    if(!isTelemetrySuppresed) {
                        telemetry.addLine("CURRENTLY ADJUSTING PIVOT");
                    }

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        manualPivotCheckpoint = (int)(pivotPosition - 150);
                        linearSlidePivot.setTargetPosition(manualPivotCheckpoint);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(-PIVOT_SPEED*0.5);
                        lightTimer.reset();
                        isStateInitialized = true;
                    }

                    // EXIT

                    if (Math.abs(pivotPosition-manualPivotCheckpoint) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.MANUAL_PIVOT_STASIS;
                    }

                    // ABORT
                    if ((gamepad1.dpad_down && !checkGOneDDOWN) ||
                        lightTimer.seconds() > 1) {
                        checkGOneDDOWN = true;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.MANUAL_PIVOT_STASIS;
                    }

                    break;

            // lower pivot slightly and reset position to 0
            // (if robot disconnected)
                // TODO: remove once reset switch works
                case MANUAL_RAISE_PIVOT:
                    if(!isTelemetrySuppresed) {
                        telemetry.addLine("CURRENTLY ADJUSTING PIVOT");
                    }

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        manualPivotCheckpoint = (int)(pivotPosition + 150);
                        linearSlidePivot.setTargetPosition(manualPivotCheckpoint);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(PIVOT_SPEED*0.5);
                        lightTimer.reset();
                        isStateInitialized = true;
                    }

                    // EXIT
                    if (Math.abs(pivotPosition-manualPivotCheckpoint) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.MANUAL_PIVOT_STASIS;
                    }


                    // ABORT
                    if ((gamepad1.dpad_up && !checkGOneDUP) ||
                        lightTimer.seconds() > 1) {
                        checkGOneDUP = true;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.MANUAL_PIVOT_STASIS;
                    }

                    break;

            // temporary pause of all function of the lift
            // (if robot disconnected)
                // TODO: remove once reset switch works
                case MANUAL_PIVOT_STASIS:
                    if (!limitSwitch) {
                        linearSlideLift.setPower(-0.4);
                    } else {
                        linearSlideLift.setPower(0);
                    }

                    if(!isTelemetrySuppresed) {
                        telemetry.addLine("CURRENTLY ADJUSTING PIVOT");
                    }
                    break;
            }

            
            // END IT ALL
            // Put the pivot at 0 and end the teleop
            if(gamepad1.dpad_left && gamepad2.dpad_left) {
                AutoInit.driveMotorTo(
                    linearSlidePivot, 
                    PIVOT_MIN_POSITION, 
                    3, 
                    -0.5
                );
                requestOpModeStop();
            }




            // TELEMETRY -----------------------------------------------------------
            if(!isTelemetrySuppresed) {
                telemetry.addData("alternate deposit", alternateDeposit);
                telemetry.addLine("RESET PIVOT: (gamepad 1) dpad_right + button_x");
                telemetry.addLine("MANUAL OVERRIDE: (gamepad 2) dpad_down + button_a");

                telemetry.addData("SLIDE STATE", linearSlideState);
                telemetry.addData("TIME", lightTimer.seconds());
                telemetry.addData("Manual Pivot Checkpoint", manualPivotCheckpoint);
                telemetry.addLine("-------------------------");

                telemetry.addLine("DRIVETRAIN");
                telemetry.addData("Front R POW", frontRight.getPower());
                telemetry.addData("Back R POW", backLeft.getPower());
                telemetry.addData("Front R POW", frontRight.getPower());
                telemetry.addData("Back L POW", backLeft.getPower());
                telemetry.addData("Drive Speed Factor", driveSpeedFactor);
                telemetry.addLine("-------------------------");

                telemetry.addLine("SLIDES");
                telemetry.addData("Lift POW", linearSlideLift.getPower());
                telemetry.addData("Lift POS", linearSlideLift.getCurrentPosition());
                telemetry.addData("Pivot POW", linearSlidePivot.getPower());
                telemetry.addData("Pivot POS", linearSlidePivot.getCurrentPosition());
                telemetry.addLine("-------------------------");

                telemetry.addLine("SERVOS");
                telemetry.addData("Intake WR POW", intakeWheelR.getPower());
                telemetry.addData("Intake WL POW", intakeWheelL.getPower());
                telemetry.addData("Intake PIVOT POS", intakePivot.getPosition());
                telemetry.addLine("-------------------------");

                telemetry.addLine("SENSORS");
                telemetry.addData("Limit Switch Activated", linearSlideSwitch.isPressed());
                telemetry.addData("Sample Sensor Gain", sampleSensor.getGain());
                telemetry.addData("Sample DIST (CM)", sampleSensor.getDistance(DistanceUnit.CM));
                telemetry.addLine("(operating range 1-10 centimeters)");
                telemetry.addData("Red", sampleSensor.getNormalizedColors().red);
                telemetry.addData("Green", sampleSensor.getNormalizedColors().green);
                telemetry.addData("Blue", sampleSensor.getNormalizedColors().blue);


                telemetry.update();
            }

        }
    }



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
     * Determines what power to apply to the slides specified by the mode the lift is in.
     * @param linearSlidePosition current position of the slide
     * @param intake use extension cushion for intake, or deposit
     * @return a correct and safe power that can be applied to the linear slide motor
     */
    public double calculateSlidePower(double linearSlidePosition, boolean intake) {
        double linearSlidePower = 0;
        double linearSlideCushion = 1;

        // determine the cushion for the linear slide so robot does not exceed extension limit
        if (intake) {
            // intake
            linearSlideCushion = (SLIDE_CONSTANTS.extensionLimit - linearSlidePosition)
                    /SLIDE_CONSTANTS.cushionRatio;
        } else if (alternateDeposit) {
            // angled deposit
            linearSlideCushion = (SLIDE_CONSTANTS.topBucketHeightAlternate - linearSlidePosition)
                    /SLIDE_CONSTANTS.cushionRatio;
        } else {
            // vertical deposit
            linearSlideCushion = (SLIDE_CONSTANTS.topBucketHeight - linearSlidePosition)
                    /SLIDE_CONSTANTS.cushionRatio;
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
        if (!intake) {
            // apply a coefficient to fight gravity
            linearSlidePower+=SLIDE_CONSTANTS.gravityCoefficient;
        }

        return linearSlidePower;
    }


    /**
     * Prevents pivot from bending the robot in half and exceeding the extension limit
     * when running the pivot manually
     * @param pivotPosition current position of the pivot
     * @return a power suitable for safe pivot use
     */
    public double calculateManualPivotPower(double pivotPosition) {
        double pivotPower = 0;
        double pivotCushion = 1;

        // find pivot cushion so it does not bend the robot in half
        // if ((-gamepad2.left_stick_y) > 0) {
        //     // for moving pivot up
        //     // END POSITION MAX
        //     pivotCushion = ((double)PIVOT_MAX_POSITION - pivotPosition)/PIVOT_CONSTANTS.cushionRatio;

        // } else {
        //     // for moving pivot down
        //     // END POSITION 0
        //     pivotCushion = (pivotPosition - (double)PIVOT_MIN_POSITION)/PIVOT_CONSTANTS.cushionRatio;

        // }

        // set power
        pivotPower = (-gamepad2.left_stick_y)*PIVOT_SPEED*pivotCushion;
        return pivotPower;
    }



    public void initHardware() {

        // HARDWARE CONFIGURATION
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        linearSlideLift = hardwareMap.get(DcMotorEx.class, "linearSlideLift");
        linearSlidePivot = hardwareMap.get(DcMotorEx.class, "linearSlidePivot");

        intakeWheelR = hardwareMap.get(CRServo.class, "intakeWheelR");
        intakeWheelL = hardwareMap.get(CRServo.class, "intakeWheelL");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        sampleSensor = hardwareMap.get(ColorRangeSensor.class, "sampleSensor");
        linearSlideSwitch = hardwareMap.get(TouchSensor.class, "linearSlideSwitch");
        pivotResetSwitch = hardwareMap.get(TouchSensor.class, "pivotResetSwitch");


        // MOTOR/SERVO DIRECTIONS AND POSITION INITIALIZATION
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideLift.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlidePivot.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sampleSensor.setGain(SENSOR_VARIABLES.sampleSensorGain);

        intakeWheelL.setDirection(DcMotorSimple.Direction.REVERSE);



    }

}
