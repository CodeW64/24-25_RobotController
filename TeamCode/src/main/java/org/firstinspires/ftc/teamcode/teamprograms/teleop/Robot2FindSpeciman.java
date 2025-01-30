package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "AAA")
@Config
public class Robot2FindSpeciman extends LinearOpMode {

    DcMotorEx linearSlideRight, linearSlideLeft;
    DcMotorEx linearPivotRight, linearPivotLeft;
    Servo specimenGrabberR, specimenGrabberL;

    public static class SpecimanValues {
        public double specimenGrabberOpenPos = 0.55; // adjust
        public double specimenGrabberClosePos = 0.625; // adjust
    }
    public static SpecimanValues SPECIMAN_VALUES = new SpecimanValues();

    @Override
    public void runOpMode() {

        linearSlideRight = hardwareMap.get(DcMotorEx.class, "linearSlideRight");
        linearSlideLeft = hardwareMap.get(DcMotorEx.class, "linearSlideLeft");
        linearSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearPivotRight = hardwareMap.get(DcMotorEx.class, "linearPivotRight");
        linearPivotLeft = hardwareMap.get(DcMotorEx.class, "linearPivotLeft");
        linearPivotRight.setDirection(DcMotorSimple.Direction.REVERSE);

        linearPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearPivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        specimenGrabberR = hardwareMap.get(Servo.class, "specimenGrabberR");
        specimenGrabberL = hardwareMap.get(Servo.class, "specimenGrabberL");

        specimenGrabberR.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            if (gamepad2.right_bumper) {
                specimenGrabberR.setPosition(SPECIMAN_VALUES.specimenGrabberClosePos);
                specimenGrabberL.setPosition(SPECIMAN_VALUES.specimenGrabberClosePos);
            } else if (gamepad2.left_bumper) {
                specimenGrabberR.setPosition(SPECIMAN_VALUES.specimenGrabberOpenPos);
                specimenGrabberL.setPosition(SPECIMAN_VALUES.specimenGrabberOpenPos);
            }


            telemetry.addData("Slide R POS", linearSlideRight.getCurrentPosition());
            telemetry.addData("Slide L POS", linearSlideLeft.getCurrentPosition());
            telemetry.addLine("--------------------------------");
            telemetry.addData("Pivot R POS", linearPivotRight.getCurrentPosition());
            telemetry.addData("Pivot L POS", linearPivotLeft.getCurrentPosition());
            telemetry.addLine("--------------------------------");
            telemetry.addData("Specimen R POS", specimenGrabberR.getPosition());
            telemetry.addData("Specimen L POS", specimenGrabberL.getPosition());
            telemetry.update();
        }

    }

}
