package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(group = "AAA")
@Config
public class Robot2TestIntake extends LinearOpMode {

    CRServo intakeWheelR, intakeWheelL;
    Servo intakePivot;
    DcMotorEx linearSlideRight, linearSlideLeft;

    ColorRangeSensor sampleSensor;

    public static class SensorVariables {
        public float sampleSensorGain = 1.0f;
        public double sampleDistance = 3.2;
    }

    public static SensorVariables SENSOR_VARIABLES = new SensorVariables();

    public static class ServoValues {
        public double pivotIntakePos = 0.45;
        public double pivotEjectSamplePos = 0.6;
        public double pivotDepositPos = 0.4;
        public double pivotRestPos = 0.52;
        public double pivotCarryPos = 0.6;
//        public double pivotHangPos = 0.2;
    }
    public static ServoValues SERVO_VALUES = new ServoValues();

    final double INTAKE_POWER_MAX = 1.0;
    final double INTAKE_POWER_HOLD = 0.06;
    final double INTAKE_POWER_EMPTY = -0.3;
    final double INTAKE_POWER_ZERO = 0;

    final double SLIDE_SPEED = 0.5;

    public enum intakeStates {
        INTAKE_ACTIVE, INTAKE_ATTEMPT_SAMPLE, INTAKE_EMPTY, DEPOSIT_ACTIVE,
        INTAKE_FULL
    }

    intakeStates intakeState = intakeStates.INTAKE_ACTIVE;

    @Override
    public void runOpMode() {

        initHardware();

        // variables
        boolean isStateInitialized = false;

        boolean checkGTwoB = true;
        boolean checkGTwoA = true;
        boolean checkGTwoX = true;
        boolean checkGTwoY = true;

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // button checks
            if (!gamepad2.b) checkGTwoB = false;
            if (!gamepad2.a) checkGTwoA = false;
            if (!gamepad2.x) checkGTwoX = false;
            if (!gamepad2.y) checkGTwoY = false;

            // run the intake
            switch (intakeState) {

            // intake ready to attempt sample in this mode
            // serves as base for all other states in this test mode
                case INTAKE_ACTIVE:
                    if (!isStateInitialized) {

                        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
                        intakeWheelR.setPower(INTAKE_POWER_ZERO);
                        intakeWheelL.setPower(INTAKE_POWER_ZERO);

                        isStateInitialized = true;
                    }

                    // EXIT

                    // go to attempt sample
                    if (gamepad2.a && !checkGTwoA) {
                        checkGTwoA = true;
                        isStateInitialized = false;
                        intakeState = intakeStates.INTAKE_ATTEMPT_SAMPLE;
                    }

                    // go to empty
                    if (gamepad2.b && !checkGTwoB) {
                        checkGTwoB = true;
                        isStateInitialized = false;
                        intakeState = intakeStates.INTAKE_EMPTY;
                    }

                    // go to deposit
                    if (gamepad2.x && !checkGTwoX) {
                        checkGTwoX = true;
                        isStateInitialized = false;
                        intakeState = intakeStates.DEPOSIT_ACTIVE;
                    }

                    // go to carry position
                    if (gamepad2.y && !checkGTwoY) {
                        checkGTwoY = true;
                        isStateInitialized = false;
                        intakeState = intakeStates.INTAKE_FULL;
                    }

                    break;

            // move intake down to grab sample
                case INTAKE_ATTEMPT_SAMPLE:
                    if (!isStateInitialized) {
                        intakeWheelR.setPower(INTAKE_POWER_MAX);
                        intakeWheelL.setPower(INTAKE_POWER_MAX);
                        intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);

                        isStateInitialized = true;
                    }


                    // EXIT

                    // go back to intake active
                    if (gamepad2.a && !checkGTwoA) {
                        checkGTwoA = true;
                        isStateInitialized = false;
                        intakeState = intakeStates.INTAKE_ACTIVE;
                    }



                    break;

            // move intake up to eject sample
                case INTAKE_EMPTY:
                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotEjectSamplePos);
                        intakeWheelR.setPower(INTAKE_POWER_EMPTY);
                        intakeWheelL.setPower(INTAKE_POWER_EMPTY);

                        isStateInitialized = true;
                    }


                    // EXIT

                    // go back to intake active
                    if (gamepad2.b && !checkGTwoB) {
                        checkGTwoB = true;
                        isStateInitialized = false;
                        intakeState = intakeStates.INTAKE_ACTIVE;
                    }
                    break;

            // move intake to deposit position (similar to intake attempt sample)
                case DEPOSIT_ACTIVE:
                    if (!isStateInitialized) {
                        intakePivot.setPosition(SERVO_VALUES.pivotDepositPos);
                        intakeWheelR.setPower(INTAKE_POWER_HOLD);
                        intakeWheelL.setPower(INTAKE_POWER_HOLD);

                        isStateInitialized = true;
                    }


                    // EXIT

                    // go back to intake active
                    if (gamepad2.x && !checkGTwoX) {
                        checkGTwoX = true;
                        isStateInitialized = false;
                        intakeState = intakeStates.INTAKE_ACTIVE;
                    }
                    break;



                case INTAKE_FULL:
                    if (!isStateInitialized) {
                        intakeWheelR.setPower(INTAKE_POWER_HOLD);
                        intakeWheelL.setPower(INTAKE_POWER_HOLD);
                        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);

                        isStateInitialized = true;
                    }

                    // EXIT

                    // go back to intake active
                    if (gamepad2.y && !checkGTwoY) {
                        checkGTwoY = true;
                        isStateInitialized = false;
                        intakeState = intakeStates.INTAKE_ACTIVE;
                    }
                    break;

            } // end intake state machine



            // run slides for easier testing
            double slidePower = (-gamepad2.right_stick_y) * SLIDE_SPEED;
            linearSlideRight.setPower(slidePower);
            linearSlideLeft.setPower(slidePower);


            // collect values from color sensor
            double sampleDist = sampleSensor.getDistance(DistanceUnit.CM);
            NormalizedRGBA sampleColors = sampleSensor.getNormalizedColors();



            // display telemetry
            telemetry.addData("state", intakeState);
            telemetry.addLine("-------------------------");
            telemetry.addLine("INTAKE PIVOT");
            telemetry.addData("pivot position", intakePivot.getPosition());
            telemetry.addLine("-------------------------");
            telemetry.addLine("INTAKE WHEELS");
            telemetry.addData("wheel R power", intakeWheelR.getPower());
            telemetry.addData("wheel L power", intakeWheelL.getPower());
            telemetry.addLine("-------------------------");
            telemetry.addLine("LINEAR SLIDES");
            telemetry.addData("slide power", slidePower);
            telemetry.addData("slide R position", linearSlideRight.getCurrentPosition());
            telemetry.addData("slide L position", linearSlideLeft.getCurrentPosition());
            telemetry.addLine("-------------------------");
            telemetry.addLine("COLOR SENSOR");
            telemetry.addData("HAS SAMPLE", isPossessingSample(sampleDist));
//            if (isPossessingSample(sampleDist)) {
//                // add functionality for color detection
//            }
            telemetry.addData("DIST (CM)", sampleDist);
            telemetry.addData("RED", sampleColors.red);
            telemetry.addData("GREEN", sampleColors.green);
            telemetry.addData("BLUE", sampleColors.blue);
            telemetry.update();

        }
    }



    public boolean isPossessingSample(double currentSampleDistance) {

        // robot has successfully acquired a sample
        if (currentSampleDistance < SENSOR_VARIABLES.sampleDistance) {
            return true;
        } else {
            // robot did not get sample
            return false;
        }
    }


    public void initHardware() {

        intakeWheelR = hardwareMap.get(CRServo.class, "intakeWheelR");
        intakeWheelL = hardwareMap.get(CRServo.class, "intakeWheelL");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        intakeWheelR.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideRight = hardwareMap.get(DcMotorEx.class, "linearSlideRight");
        linearSlideLeft = hardwareMap.get(DcMotorEx.class, "linearSlideLeft");

        linearSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sampleSensor = hardwareMap.get(ColorRangeSensor.class, "sampleSensor");

        sampleSensor.setGain(SENSOR_VARIABLES.sampleSensorGain);
    }
}
