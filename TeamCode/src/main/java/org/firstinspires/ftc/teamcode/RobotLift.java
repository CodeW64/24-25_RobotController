package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Contains all actions necessary to run the lift of the robot with Road Runner. <br> <br>
 * <strong>NOTE:</strong>
 * Use the nested class instances to access the actions within them.
 */
public class RobotLift {


    private DcMotorEx linearSlideLift, linearSlidePivot;
    private CRServo intakeWheelR, intakeWheelL;
    private Servo intakePivot;
    private CRServo duckSpinner;
    private ColorRangeSensor sampleSensor;
    private TouchSensor linearSlideSwitch;
    private final Telemetry telemetry;

    public Intake intake;
    public Slide slide;
    public Pivot pivot;
    public Duck duck;

    public RobotLift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // hardware map
        linearSlideLift = hardwareMap.get(DcMotorEx.class, "linearSlideLift");
        linearSlidePivot = hardwareMap.get(DcMotorEx.class, "linearSlidePivot");

        intakeWheelR = hardwareMap.get(CRServo.class, "intakeWheelR");
        intakeWheelL = hardwareMap.get(CRServo.class, "intakeWheelL");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        duckSpinner = hardwareMap.get(CRServo.class, "duckSpinner");

        sampleSensor = hardwareMap.get(ColorRangeSensor.class, "sampleSensor");
        linearSlideSwitch = hardwareMap.get(TouchSensor.class, "linearSlideSwitch");

        // direction changes
        linearSlideLift.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlidePivot.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeWheelL.setDirection(DcMotorSimple.Direction.REVERSE);

        // encoder initialization
        linearSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // action class initialization
        intake = new Intake();
        slide = new Slide();
        pivot = new Pivot();
        duck = new Duck();
    }


    public class Intake {

    }




    public class Slide {

    }



    public class Pivot {

    }


    public class Duck {

        public Action spinDuck() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    duckSpinner.setPower(-0.15);
                    return false;
                }
            };
        }

    }




}
