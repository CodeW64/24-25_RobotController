package org.firstinspires.ftc.teamcode.teamprograms.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.teamprograms.teleop.IntoTheDeepTeleop;

@Autonomous(name = "The S.S. Steve's Primitive Net of Butter", group = "CWA")
public class IntoTheDeepBuckets extends LinearOpMode {

    // HARDWARE
    DcMotorEx frontRight, backRight, frontLeft, backLeft;
    DcMotorEx linearSlideLift, linearSlidePivot;
    CRServo intakeWheelR, intakeWheelL;
    Servo intakePivot;

    ColorRangeSensor sampleSensor;
    TouchSensor linearSlideSwitch, pivotResetSwitch;


    // SERVO POSITION VALUES (editable by FTC dashboard)
    public static class ServoValues {
        public double pivotIntakePos = 0.44;
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
        public int extensionLimit = 2400; // for intake
        public int topBucketHeight = 3800;
        public int bottomBucketHeight = 1900;
        public int topBucketHeightAlternate = 4200;
        public int bottomBucketHeightAlternate = 2200;

    }
    public static SlideConstants SLIDE_CONSTANTS = new SlideConstants();

    // PIVOT VARIABLES
    public static class PivotConstants {
        public int hangPos = 3700;
        public int attemptSamplePos = 100;
        public int intakeRestPos = 400;
        public int maxPos = 3600;
        public int minPos = 0;
        public int alternateDepositPos = 2850;
        public int alternateRetractSetPos = 3200;
    }
    public static PivotConstants PIVOT_CONSTANTS = new PivotConstants();

    final double SLIDE_SPEED = 0.9;
    final double PIVOT_SPEED = 1.0;


    // AUTONOMOUS VARIABLES

    int positionFR = 0;
    int positionBR = 0;
    int positionFL = 0;
    int positionBL = 0;

    int positionSL = 0;

    boolean limitSwitch = false;



    @Override
    public void runOpMode() {

        initHardware();

        telemetry.addLine("Press Start");
        telemetry.update();
        waitForStart();

        // for preload
        intakeWheelR.setPower(INTAKE_POWER_HOLD);
        intakeWheelL.setPower(INTAKE_POWER_HOLD);
        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);

        // AUTONOMOUS ROUTINE ------------------------------------------------------------------

        // MOVE PIVOT TO PLACE POS AND DRIVE TO BUCKET

        linearSlidePivot.setPower(1.0);
        linearSlidePivot.setTargetPosition(PIVOT_CONSTANTS.alternateDepositPos);
        setDrivePower(0.4);
        directionSlideRight(700);

        sleep(2000);

//        setDrivePower(0.4);
        directionForward(500);

//        frontRight.setPower(0.4);
//        backRight.setPower(0.4);
//        frontLeft.setPower(0.4);
//        backLeft.setPower(0.4);

//        frontRight.setTargetPosition(500);
//        backRight.setTargetPosition(500);
//        frontLeft.setTargetPosition(500);
//        backLeft.setTargetPosition(500);
        addTelemetry();

        sleep(1000);
        sleep(30000);

        // MOVE SLIDES UP AND DEPOSIT

        setSlidePower(0.5);
        linearSlideLift.setTargetPosition(SLIDE_CONSTANTS.topBucketHeightAlternate);

        sleep(3000);

        intakePivot.setPosition(SERVO_VALUES.pivotAlternateDepositPos);
        sleep(500);
        intakeWheelR.setPower(INTAKE_POWER_EMPTY);
        intakeWheelL.setPower(INTAKE_POWER_EMPTY);
        sleep(1000);

        // MOVE SLIDES DOWN SAFELY

        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
        linearSlidePivot.setTargetPosition(PIVOT_CONSTANTS.alternateRetractSetPos);

        sleep(1000);

        slideDisableRUNTO();
        while (!limitSwitch && opModeIsActive()) {
            limitSwitch = linearSlideSwitch.isPressed();
            setSlidePower(-0.8);
        }
        slideEnableRUNTO();










    }


    /*----------------------------------


   MISCELLANEOUS


    ----------------------------------*/

    public void setDrivePower(double power) {

        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power);


    }

    public void setSlidePower(double power) {
        linearSlideLift.setPower(power);
    }


    public void addTelemetry() {

        telemetry.addData("FR CURRENT POS", frontRight.getCurrentPosition());
        telemetry.addData("BR CURRENT POS", backRight.getCurrentPosition());
        telemetry.addData("FL CURRENT POS", frontLeft.getCurrentPosition());
        telemetry.addData("BL CURRENT POS", backLeft.getCurrentPosition());

        telemetry.addLine("-----------------------------------------");
        telemetry.addData("FR TARGET", frontRight.getTargetPosition());
        telemetry.addData("BR TARGET", backRight.getTargetPosition());
        telemetry.addData("FL TARGET", frontLeft.getTargetPosition());
        telemetry.addData("BL TARGET", backLeft.getTargetPosition());

        telemetry.addLine("-----------------------------------------");
        telemetry.addData("FR POW", frontRight.getPower());
        telemetry.addData("BR POW", backRight.getPower());
        telemetry.addData("FL POW", frontLeft.getPower());
        telemetry.addData("BL POW", backLeft.getPower());

        telemetry.addLine("-----------------------------------------");

        telemetry.update();

    }

    /*----------------------------------


   SLIDE/PIVOT MOVEMENT


    ----------------------------------*/


    /*public void slidePositionForward(int positionChange) {

        positionSL+=positionChange;

        linearSlideLift.setTargetPosition(positionSL);

    }

    public void slidePositionBackward(int positionChange) {
        positionSL-=positionChange;
        linearSlideLift.setTargetPosition(positionSL);

    }*/

    /**
     * for use after moving slide down by detecting limit switch
     */
    public void slidePositionReset() {
        positionSL = 10;
        linearSlideLift.setTargetPosition(10);

    }


    /**
     * also sets slide power to 0
     */
    public void slideEnableRUNTO() {
        linearSlideLift.setPower(0);
        linearSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLift.setTargetPosition(0);
        linearSlideLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * also sets slide power to 0
     */
    public void slideDisableRUNTO() {
        linearSlideLift.setPower(0);
        linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /*----------------------------------


   AUTONOMOUS MOVEMENT METHODS


    ----------------------------------*/


    public void directionForward(int positionChange) {

        positionFR+=positionChange;
        positionBR+=positionChange;
        positionFL+=positionChange;
        positionBL+=positionChange;



        frontRight.setTargetPosition(positionFR);
        backRight.setTargetPosition(positionBR);
        frontLeft.setTargetPosition(positionFL);
        backLeft.setTargetPosition(positionBL);

    }

    public void directionBackward(int positionChange) {

        positionFR-=positionChange;
        positionBR-=positionChange;
        positionFL-=positionChange;
        positionBL-=positionChange;


        frontRight.setTargetPosition(positionFR);
        backRight.setTargetPosition(positionBR);
        frontLeft.setTargetPosition(positionFL);
        backLeft.setTargetPosition(positionBL);

    }

    public void directionTurnRight(int positionChange) {

        positionFR-=positionChange;
        positionBR-=positionChange;
        positionFL+=positionChange;
        positionBL+=positionChange;


        frontRight.setTargetPosition(positionFR);
        backRight.setTargetPosition(positionBR);
        frontLeft.setTargetPosition(positionFL);
        backLeft.setTargetPosition(positionBL);

    }

    public void directionTurnLeft(int positionChange) {

        positionFR+=positionChange;
        positionBR+=positionChange;
        positionFL-=positionChange;
        positionBL-=positionChange;


        frontRight.setTargetPosition(positionFR);
        backRight.setTargetPosition(positionBR);
        frontLeft.setTargetPosition(positionFL);
        backLeft.setTargetPosition(positionBL);

    }

    public void directionPivotRight(int positionChange) {



        positionFL+=positionChange;
        positionBL+=positionChange;



        frontLeft.setTargetPosition(positionFL);
        backLeft.setTargetPosition(positionBL);

    }

    public void directionPivotLeft(int positionChange) {

        positionFR+=positionChange;
        positionBR+=positionChange;


        frontRight.setTargetPosition(positionFR);
        backRight.setTargetPosition(positionBR);
    }



    public void directionSlideRight(int positionChange) {

        positionFR-=positionChange;
        positionBR+=positionChange;
        positionFL+=positionChange;
        positionBL-=positionChange;


        frontRight.setTargetPosition(positionFR);
        backRight.setTargetPosition(positionBR);
        frontLeft.setTargetPosition(positionFL);
        backLeft.setTargetPosition(positionBL);
    }


    public void directionSlideLeft(int positionChange) {

        positionFR+=positionChange;
        positionBR-=positionChange;
        positionFL-=positionChange;
        positionBL+=positionChange;


        frontRight.setTargetPosition(positionFR);
        backRight.setTargetPosition(positionBR);
        frontLeft.setTargetPosition(positionFL);
        backLeft.setTargetPosition(positionBL);

    }


    /**
     * for use when robot is set drive power to run into the wall
     */
    public void localizeDrivetrain() {

        positionFR = frontRight.getCurrentPosition();
        positionBR = backRight.getCurrentPosition();
        positionFL = frontLeft.getCurrentPosition();
        positionBL = backLeft.getCurrentPosition();

        frontRight.setTargetPosition(positionFR);
        backRight.setTargetPosition(positionBR);
        frontLeft.setTargetPosition(positionFL);
        backLeft.setTargetPosition(positionBL);

    }


    /*----------------------------------


   INITIALIZATION METHODS


    ----------------------------------*/

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
        linearSlideLift.setTargetPosition(0);
        linearSlideLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlidePivot.setTargetPosition(0);
        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setTargetPosition(0);
        backRight.setTargetPosition(0);
        frontLeft.setTargetPosition(0);
        backLeft.setTargetPosition(0);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sampleSensor.setGain(SENSOR_VARIABLES.sampleSensorGain);

        intakeWheelL.setDirection(DcMotorSimple.Direction.REVERSE);



    }
}
