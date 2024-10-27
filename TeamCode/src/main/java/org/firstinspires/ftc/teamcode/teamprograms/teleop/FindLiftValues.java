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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(group = "Z")
public class FindLiftValues extends LinearOpMode {

    DcMotorEx linearSlideLift, linearSlidePivot;
    Servo intakePivot;

    CRServo intakeWheelR, intakeWheelL;

    ColorRangeSensor sampleSensor;

    // SERVO POSITION VALUES (editable by FTC dashboard)
    public static class ServoValues {
        public double pivotIntakePos = 0.44;
        public double pivotDepositPos = 0.85;
        public double pivotRestPos = 0.52;

        public double intakePowerMax = 1.0;

        public double intakePowerRest = 0.06;

        public double intakePowerEmpty = -0.7;
    }
    public static ServoValues SERVO_VALUES = new ServoValues();

    // SENSOR VARIABLES (editable by FTC dashboard)
    public static class SensorVariables {
        public float sampleSensorGain = 1.0f;
        public double sampleDistance = 2.4;
        public double sampleCodeBlue = 0.007;
    }
    public static SensorVariables SENSOR_VARIABLES = new SensorVariables();



    // worm gear max position 5200
    // slide max position 2400

    // HAVE WHEELS REST AT 0 POWER WHEN NOT DOING ANYTHING
    // distance from color sensor to sample < 2.2 CM (when acquired)

    @Override
    public void runOpMode() {

        linearSlideLift = hardwareMap.get(DcMotorEx.class, "linearSlideLift");
        linearSlidePivot = hardwareMap.get(DcMotorEx.class, "linearSlidePivot");

        linearSlideLift.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlidePivot.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        intakeWheelR = hardwareMap.get(CRServo.class, "intakeWheelR");
        intakeWheelL = hardwareMap.get(CRServo.class, "intakeWheelL");

        intakeWheelL.setDirection(DcMotorSimple.Direction.REVERSE);

        sampleSensor = hardwareMap.get(ColorRangeSensor.class, "sampleSensor");
        sampleSensor.setGain(SENSOR_VARIABLES.sampleSensorGain);

        telemetry.addLine("Press Start");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            if (gamepad2.a) {
                intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
            } else if (gamepad2.x) {
                intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);
            } else if (gamepad2.b) {
                intakePivot.setPosition(SERVO_VALUES.pivotDepositPos);
            }

            if (gamepad2.dpad_down) {
                intakeWheelR.setPower(SERVO_VALUES.intakePowerRest);
                intakeWheelL.setPower(SERVO_VALUES.intakePowerRest);
            } else if (gamepad2.dpad_left) {
                intakeWheelR.setPower(SERVO_VALUES.intakePowerMax);
                intakeWheelL.setPower(SERVO_VALUES.intakePowerMax);
            } else if (gamepad2.dpad_right) {
                intakeWheelR.setPower(SERVO_VALUES.intakePowerEmpty);
                intakeWheelL.setPower(SERVO_VALUES.intakePowerEmpty);
            }


            if (gamepad2.y) {
                sampleSensor.setGain(sampleSensor.getGain()+1);
            } else if (gamepad2.dpad_up) {
                sampleSensor.setGain(sampleSensor.getGain()-1);
            }

            telemetry.addData("distance CM", sampleSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance INCH", sampleSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("red", sampleSensor.getNormalizedColors().red);
            telemetry.addData("green", sampleSensor.getNormalizedColors().green);
            telemetry.addData("blue", sampleSensor.getNormalizedColors().blue);
            telemetry.addData("gain", sampleSensor.getGain());
            telemetry.addData("blue (not normalized)", sampleSensor.blue());
            telemetry.addData("argb", sampleSensor.argb());
            telemetry.addData("alpha", sampleSensor.alpha());
            telemetry.addData("light detected", sampleSensor.getLightDetected());

            telemetry.addLine("------------------------------");

            telemetry.addData("SLIDE POS", linearSlideLift.getCurrentPosition());
            telemetry.addData("WORM GEAR POS", linearSlidePivot.getCurrentPosition());
            telemetry.update();


        }


    }
}
