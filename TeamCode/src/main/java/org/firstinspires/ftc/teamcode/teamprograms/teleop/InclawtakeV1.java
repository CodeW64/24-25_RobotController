package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(group = "A")
public class InclawtakeV1 extends LinearOpMode {

//    Servo intakePivot, intakeWheelR, intakeWheelL;
    CRServo intakeWheelR, intakeWheelL;
    ColorRangeSensor sampleSensor;


    // logic variable checks




    @Override
    public void runOpMode() {

        // initialize actuators and sensors
//        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakeWheelR = hardwareMap.get(CRServo.class, "intakeWheelR");
        intakeWheelL = hardwareMap.get(CRServo.class, "intakeWheelL");

        intakeWheelL.setDirection(DcMotorSimple.Direction.REVERSE);

        sampleSensor = hardwareMap.get(ColorRangeSensor.class, "sampleSensor");

        telemetry.addLine("CURRENTLY TESTING:");
        telemetry.addLine("sampleSensor");
        telemetry.addLine("--------------------------------");
        telemetry.addLine("Press Start");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            // SERVOS
            double intakePower = gamepad1.right_stick_y;
            intakeWheelR.setPower(intakePower);
            intakeWheelL.setPower(intakePower);

            // hold sample in intake by applying small power to wheels





            // increase/decrease gain of RGB colors on color sensor
            if (gamepad1.dpad_up) {
                sampleSensor.setGain(sampleSensor.getGain()+1);
            } else if (gamepad1.dpad_down) {
                sampleSensor.setGain(sampleSensor.getGain()-1);
            }

            telemetry.addData("distance MM", sampleSensor.getDistance(DistanceUnit.MM));
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

            telemetry.addData("Intake Wheel Power", intakePower);
            telemetry.update();



        }



    }


}
