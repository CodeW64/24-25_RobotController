package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "Z")
public class FindLevel3Ascent extends LinearOpMode {

    DcMotorEx linearSlideLift, linearSlidePivot;
    DcMotorEx linearActuatorRight, linearActuatorLeft;

    private final double SLIDE_EXTENSION_MAX = 4200;
    private final double CUSHION_RATIO = 400;
    private final double GRAVITY_COEFFICIENT = 0.0005;


    private final double ACTUATOR_SPEED = 0.5;
    private final double SLIDE_SPEED = 0.7;
    private final double PIVOT_SPEED = 0.9;


    @Override
    public void runOpMode() {

        linearSlideLift = hardwareMap.get(DcMotorEx.class, "linearSlideLift");
        linearSlidePivot = hardwareMap.get(DcMotorEx.class, "linearSlidePivot");

        linearActuatorRight = hardwareMap.get(DcMotorEx.class, "linearActuatorRight");
        linearActuatorLeft = hardwareMap.get(DcMotorEx.class, "linearActuatorLeft");

        linearSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearSlideLift.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlidePivot.setDirection(DcMotorSimple.Direction.REVERSE);

        linearActuatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // get positions
            double linearSlidePosition = linearSlideLift.getCurrentPosition();


            // move linear actuators
            double powerActuatorUp = (-gamepad2.right_trigger) * ACTUATOR_SPEED;
            double powerActuatorDown = (-gamepad2.left_trigger) * ACTUATOR_SPEED;
            linearActuatorRight.setPower(powerActuatorUp - powerActuatorDown);

            // move linear slide
            double powerSlide = calculateSlidePower(linearSlidePosition);
            linearSlideLift.setPower(powerSlide);

            // move pivot
            double powerPivot = (-gamepad2.left_stick_y) * PIVOT_SPEED;
            linearSlidePivot.setPower(powerPivot);



            telemetry.addLine("CONTROLS (G2)");
            telemetry.addLine("L/R triggers: linear actuators");
            telemetry.addLine("Right stick: linear slide");
            telemetry.addLine("Left stick: pivot");
            telemetry.addLine("--------------------------------");
            telemetry.addData("Lift Position", linearSlideLift.getCurrentPosition());
            telemetry.addData("Pivot Position", linearSlidePivot.getCurrentPosition());
            telemetry.addData("LAR Position", linearActuatorRight.getCurrentPosition());
            telemetry.addData("LAL Position", linearActuatorLeft.getCurrentPosition());
            telemetry.addLine("--------------------------------");
            telemetry.addData("LA UP POW", powerActuatorUp);
            telemetry.addData("LA DOWN POW", powerActuatorDown);
            telemetry.addData("SLIDE POW", powerSlide);
            telemetry.addData("PIVOT POW", powerPivot);
            telemetry.update();
        }
    }


    public double calculateSlidePower(double linearSlidePosition) {
        double linearSlidePower = 0;

        // find cushion for slide to not over-extend and cause belt to slip
        double linearSlideCushion = (SLIDE_EXTENSION_MAX - linearSlidePosition)
                /CUSHION_RATIO;

        // determine whether to apply the cushion or ignore it
        if ((-gamepad2.right_stick_y) > 0) {
            // lift going up, use cushion
            linearSlidePower = (-gamepad2.right_stick_y)*SLIDE_SPEED*linearSlideCushion;
        } else {
            // lift going down, ignore cushion
            linearSlidePower = (-gamepad2.right_stick_y)*SLIDE_SPEED;
        }

        // make descent of slides slightly slower so it is not jarring
        if ((-gamepad2.right_stick_y) < 0) {
            linearSlidePower*=0.9;
        }


        // apply a coefficient to fight gravity
        linearSlidePower+=GRAVITY_COEFFICIENT;

        return linearSlidePower;
    }

}
