package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
 * Teleop Version: 0.3.2 BETA
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
 * --------------------------
 * Gamepad 2 (lift)
 * [right_stick_y] - extend/retract slides
 * [left_stick_y] - pivot slides (only in HANG_TIME)
 * [right_bumper] - set intake wheels to high speed (intake)
 * [right_bumper] - set intake wheels to empty (deposit)
 * [left_bumper] - set intake wheels to zero to rest (intake)
 * [left_bumper] - set intake wheels to low speed (deposit)
 * [b_button] - reverse direction of intake wheels (intake)
 * [left_trigger] - switch from intake to deposit mode (deposit mode: protect/deposit servo position)
 * [right_trigger] - switch from deposit to intake mode (intake mode: attempt sample grab)
 * [x_button/y_button] - override limit switch
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

@TeleOp(name = "The S.S. Steve's Maiden Voyage", group = "A")
@Config
public class IntoTheDeepTeleop extends LinearOpMode {


    // HARDWARE
    DcMotorEx frontRight, backRight, frontLeft, backLeft;
    DcMotorEx linearSlideLift, linearSlidePivot;
    CRServo intakeWheelR, intakeWheelL;
    Servo intakePivot;

    ColorRangeSensor sampleSensor;
    TouchSensor linearSlideSwitch;

    // SERVO POSITION VALUES (editable by FTC dashboard)
    public static class ServoValues {
        public double pivotIntakePos = 0.44;
        public double pivotDepositPos = 0.85;
        public double pivotRestPos = 0.52;
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
        public double sampleDistance = 2.4;
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
    }
    public static SlideConstants SLIDE_CONSTANTS = new SlideConstants();

    public static class PivotConstants {
        public double cushionRatio = 400;
    }
    public static PivotConstants PIVOT_CONSTANTS = new PivotConstants();

    final int PIVOT_MAX_POSITION = 5200;
    final int PIVOT_INTAKE_RETRACT_POSITION = 350;
    final int PIVOT_MIN_POSITION = 0;




    // ROBOT LIFT STATES
    enum LinearSlideStates {
        MANUAL_OVERRIDE,

        INTAKE_ACTIVE, INTAKE_ATTEMPT_SAMPLE, INTAKE_FULL, INTAKE_EMPTY,
        INTAKE_RETRACT_SET, INTAKE_RETRACT, INTAKE_EXTEND, PIVOT_TO_DEPOSIT,

        DEPOSIT_ACTIVE, DEPOSIT_RETRACT_SET, DEPOSIT_RETRACT, PIVOT_TO_INTAKE,
        HANG_TIME
    }

    LinearSlideStates linearSlideState;

    ElapsedTime lightTimer;


    // constant variables
    final double SLIDE_SPEED = 0.5;
    final double PIVOT_SPEED = 1.0;

    boolean tankDrive = true;


    @Override
    public void runOpMode() {

        initHardware();

        // VARIABLES/OBJECTS (miscellaneous) ----------------------------------------------------------

        // drivetrain
        int driveSpeedIndex = 2; // change this to the index of the speed you want to start at
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

        lightTimer = new ElapsedTime();

        // STATE MACHINE LOGIC

        boolean isStateInitialized = false;
        boolean isIntakeProtected = false;
        boolean isArmPositionSet = true;
        boolean overrideSampleSensor = true;

        // -----------------------------------------------------------------------------------

        lightTimer.startTime();


        while (opModeInInit()) {

            // change drive mode
            if (gamepad1.a) {
                tankDrive = true;
            } else if (gamepad1.b) {
                tankDrive = false;
            }

            // START
            telemetry.addLine("TELEOP VERSION 0.3.2 BETA");
            telemetry.addLine("-------------------------");
            telemetry.addData("COLOR SENSOR OVERRIDDEN", overrideSampleSensor);
            telemetry.addData("TANK DRIVE", tankDrive);
            telemetry.addLine("Press Start");
            telemetry.addLine("-------------------------");
            telemetry.addLine("G1 A: TANK DRIVE");
            telemetry.addLine("G1 B: MECANUM DRIVE");
            telemetry.update();
        }

        waitForStart();

        lightTimer.reset();
        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;



        // LOOP -----------------------------------------------------------------------------

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


            // LIFT ----------------------------------------------------------------------------

            // gather values
            double linearSlidePower = 0;
            double linearSlidePosition = linearSlideLift.getCurrentPosition();

            boolean linearSwitch = linearSlideSwitch.isPressed();

            double currentSampleDistance = sampleSensor.getDistance(DistanceUnit.CM);

            double pivotPosition = linearSlidePivot.getCurrentPosition();


            // enter MANUAL_OVERRIDE
            if (gamepad2.dpad_down && !checkGTwoDDOWN && gamepad2.a && !checkGTwoA
                && linearSlideState != LinearSlideStates.MANUAL_OVERRIDE) {
                checkGTwoDDOWN = true;
                checkGTwoA = true;
                linearSlideState = LinearSlideStates.MANUAL_OVERRIDE;
            }


            switch (linearSlideState) {

            // lift in position to grab samples
            // speed of intake wheels is at max
                case INTAKE_ACTIVE:

                    // FIXME: pivot currently not fully lowered as temporary fix

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isIntakeProtected = false;
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

                    // SERVOS

                    // change speed of intake wheels
                    if (gamepad2.left_bumper) {
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);
                    } else if (gamepad2.right_bumper) {
                        intakeWheelR.setPower(INTAKE_POWER_HOLD);
                        intakeWheelL.setPower(INTAKE_POWER_HOLD);
                    }

                    // move intake pivot to and from protected/default position
                    if (gamepad2.y && !checkGTwoY && !isIntakeProtected) {
                        checkGTwoY = true;
                        intakePivot.setPosition(SERVO_VALUES.pivotDepositPos);
                        isIntakeProtected = true;
                    } else if (gamepad2.y && !checkGTwoY && isIntakeProtected) {
                        checkGTwoY = true;
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        isIntakeProtected = false;
                    }

                    // FIXME: this is a temporary manual fix for color sensor
                    if (gamepad2.b) {
                        intakeWheelR.setPower(INTAKE_POWER_EMPTY);
                        intakeWheelL.setPower(INTAKE_POWER_EMPTY);
                    }

                    // attempt to grab a sample (if safe)
                    if (gamepad2.right_trigger > 0.1 && !checkGTwoRT &&
                        linearSlidePosition > 600) {
                        checkGTwoRT = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        isIntakeProtected = false;
                        linearSlideState = LinearSlideStates.INTAKE_ATTEMPT_SAMPLE;
                    }

                    // EXIT INTAKE

                    // start sequence to pivot to deposit
                    if (isArmPositionSet && gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        isIntakeProtected = false;
                        linearSlideState = LinearSlideStates.INTAKE_RETRACT_SET;
                    }
                    break;

            // robot attempts to grab a sample
            // intake cannot be messed with while in this mode
                case INTAKE_ATTEMPT_SAMPLE:

                    // FIXME: pivot currently lowering here as temporary fix

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);
                        intakeWheelR.setPower(INTAKE_POWER_MAX);
                        intakeWheelL.setPower(INTAKE_POWER_MAX);

                        linearSlidePivot.setTargetPosition(PIVOT_MIN_POSITION);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(-PIVOT_SPEED);
                        isArmPositionSet = false;
                        lightTimer.reset();
                        isStateInitialized = true;
                    }

                    // PIVOT
                    // FIXME: temporary
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
                    if (isArmPositionSet && lightTimer.seconds() > 1.5) {

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
                    } else if (isArmPositionSet && lightTimer.seconds() > 1) {
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

            // robot is in possession of a sample
            // speed of intake wheels changes to accommodate
                case INTAKE_FULL:

                    // exit immediately if color sensor has proven to be faulty
//                    if (overrideSampleSensor) {
//                        intakeWheelR.setPower(INTAKE_POWER_HOLD);
//                        intakeWheelL.setPower(INTAKE_POWER_HOLD);
//                        isStateInitialized = false;
//                        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
//                        continue;
//                    }

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

                    // TODO: remove once automatic mode has proven to work smoothly
                    if (gamepad2.y) {
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        isIntakeProtected = false;
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        linearSlideState = LinearSlideStates.INTAKE_EMPTY;
                    }

                    // EXIT INTAKE

                    // start sequence to pivot to deposit
                    if (isArmPositionSet && gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        isIntakeProtected = false;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

                    if (lightTimer.seconds() > 0.7) {
                        // exit emptying
                        linearSlideLift.setPower(0);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.INTAKE_ACTIVE;
                    } else if (lightTimer.seconds() > 0.3) {
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
                    if (Math.abs(pivotPosition-PIVOT_INTAKE_RETRACT_POSITION) < 10 &&
                        lightTimer.seconds() > 0.3) {
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
                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }

                    if (!linearSwitch) {
                        linearSlideLift.setPower(-0.8);
                    } else {
                        linearSlideLift.setPower(0);
                    }

                    // stop slides once it fully retracts
                    if (linearSwitch || gamepad2.x || gamepad2.y) {
                        linearSlideLift.setPower(0);
                        linearSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    }


                    // exit mode if slide has fully retracted
                    if (isArmPositionSet) {
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

                case INTAKE_EXTEND:
                    // TODO: for a later date if needed
                    break;

            // move lift from intake mode to deposit mode
                case PIVOT_TO_DEPOSIT:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        linearSlidePivot.setTargetPosition(PIVOT_MAX_POSITION);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(PIVOT_SPEED);
                        isIntakeProtected = true;
                        isStateInitialized = true;
                    }



                    // EXIT

                    // make intake accessible once lift has finished pivoting
                    if (Math.abs(pivotPosition-PIVOT_MAX_POSITION) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_ACTIVE;
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

            // lift in position to score samples
                case DEPOSIT_ACTIVE:

                    if (!isStateInitialized) {
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
                    break;

            // ready lift servos to retract slide
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
                    if (lightTimer.seconds() > 0.5) {
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

            // retract slide so it can pivot
                case DEPOSIT_RETRACT:

                    // TODO: eventually make slides slowly accelerate down

                    if (!linearSwitch) {
                        linearSlideLift.setPower(-0.5);
                    } else {
                        linearSlideLift.setPower(0);
                    }

                    // exit mode if slide has fully retracted;
                    // if limit switch fails, driver can override
                    if (linearSwitch || gamepad2.x || gamepad2.y) {
                        linearSlideLift.setPower(0);
                        linearSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        linearSlideState = LinearSlideStates.PIVOT_TO_INTAKE;
                    }

                    // ABORT

                    // go back to deposit if mistaken
                    if (gamepad2.left_trigger > 0.1 && !checkGTwoLT) {
                        checkGTwoLT = true;
                        linearSlideLift.setPower(0);
                        isStateInitialized = false;
                        isIntakeProtected = true;
                        linearSlideState = LinearSlideStates.DEPOSIT_ACTIVE;
                    }
                    break;

            // move lift from deposit mode to intake mode
                case PIVOT_TO_INTAKE:


                    // FIXME: should be min position, currently not for temporary fix
                    linearSlidePivot.setTargetPosition(PIVOT_INTAKE_RETRACT_POSITION);
                    linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearSlidePivot.setPower(-PIVOT_SPEED);

                    // EXIT

                    // make intake accessible once lift has finished pivoting
                    if (Math.abs(pivotPosition-PIVOT_INTAKE_RETRACT_POSITION) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        isIntakeProtected = true;
                        linearSlideState = LinearSlideStates.PIVOT_TO_DEPOSIT;
                    }
                    break;

            // allows slides to pivot manually (can enter from DEPOSIT_ACTIVE)
            // NOTE: this state is dangerous
                case HANG_TIME:

                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);
                        isStateInitialized = true;
                    }

                    // TODO: make hang time slide power more sophisticated
                    linearSlidePower = calculateSlidePower(linearSlidePosition, false);
                    linearSlideLift.setPower(linearSlidePower);

                    linearSlidePivot.setPower(calculateManualPivotPower(pivotPosition));

                    // EXIT

                    if (gamepad1.dpad_up && gamepad2.dpad_up) {
                        linearSlideLift.setPower(0);
                        linearSlidePivot.setPower(0);
                        isStateInitialized = false;
                        linearSlideState = LinearSlideStates.DEPOSIT_RETRACT_SET;
                    }
                    break;

            // in case everything else fails
            // NOTE: this state is dangerous
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
                        linearSlideState = LinearSlideStates.DEPOSIT_RETRACT_SET;
                    }

                    break;
            }




// DRIVETRAIN ------------------------------------------------------------------------------------

            // TODO: add april tag runner?


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

            // assign power and set power
            if (tankDrive) {
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




            // TELEMETRY -----------------------------------------------------------
            telemetry.addData("SLIDE STATE", linearSlideState);
            telemetry.addData("TIME", lightTimer.seconds());
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



    public double calculateSlidePower(double linearSlidePosition, boolean intake) {
        double linearSlidePower = 0;
        double linearSlideCushion = 1;

        if (intake) {
            // for intake

            // ensure robot does not exceed extension limit
            linearSlideCushion = (SLIDE_CONSTANTS.extensionLimit - linearSlidePosition)
                    /SLIDE_CONSTANTS.cushionRatio;

            // use the power cushion if robot is extending, ignore if retracting
            if ((-gamepad2.right_stick_y) > 0) {
                linearSlidePower = (-gamepad2.right_stick_y)*SLIDE_SPEED*linearSlideCushion;
            } else {
                linearSlidePower = (-gamepad2.right_stick_y)*SLIDE_SPEED;
            }
        } else {
            // for deposit
            linearSlideCushion = (SLIDE_CONSTANTS.topBucketHeight - linearSlidePosition)
                    /SLIDE_CONSTANTS.cushionRatio;

            // use the power cushion if robot is extending, ignore if retracting
            if ((-gamepad2.right_stick_y) > 0) {
                linearSlidePower = ((((-gamepad2.right_stick_y)*SLIDE_SPEED)+SLIDE_CONSTANTS.gravityCoefficient)*
                        linearSlideCushion);
            } else {
                // make lowering slides slower
                linearSlidePower = (((-gamepad2.right_stick_y)*(SLIDE_SPEED*0.75))+SLIDE_CONSTANTS.gravityCoefficient);
            }

        }

        return linearSlidePower;
    }


    public double calculateManualPivotPower(double pivotPosition) {
        double pivotPower = 0;
        double pivotCushion = 1;

        // find pivot cushion so it does not bend the robot in half
        if ((-gamepad2.left_stick_y) > 0) {
            // for moving pivot up
            // END POSITION MAX
            pivotCushion = ((double)PIVOT_MAX_POSITION - pivotPosition)/PIVOT_CONSTANTS.cushionRatio;

        } else {
            // for moving pivot down
            // END POSITION 0
            pivotCushion = (pivotPosition - (double)PIVOT_MIN_POSITION)/PIVOT_CONSTANTS.cushionRatio;

        }

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
