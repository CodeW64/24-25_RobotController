package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Robot2TestLift extends LinearOpMode {

    DcMotorEx linearSlideRight, linearSlideLeft;
    DcMotorEx linearPivotRight, linearPivotLeft;

    final double SLIDE_SPEED = 0.7;
    final double PIVOT_SPEED = 0.7;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        double slidePower = 0;
        double pivotPower = 0;

        while (opModeIsActive()) {

            double linearSlidePosition = (linearSlideRight.getCurrentPosition() + linearSlideLeft.getCurrentPosition()) / 2.0;

            // set power for motors
            slidePower = calculateSlidePower(linearSlidePosition);
            pivotPower = (-gamepad2.left_stick_y) * PIVOT_SPEED;


            // apply power to motors
            linearSlideRight.setPower(slidePower);
            linearSlideLeft.setPower(slidePower);
            linearPivotRight.setPower(pivotPower);
            linearPivotLeft.setPower(pivotPower);









// TELEMETRY --------------------------------------------------------------------------------------

            telemetry.addLine("SLIDES");
            telemetry.addData("slide R POW", linearSlideRight.getPower());
            telemetry.addData("slide L POW", linearSlideLeft.getPower());
            telemetry.addData("slide R POS", linearSlideRight.getCurrentPosition());
            telemetry.addData("slide L POS", linearSlideLeft.getCurrentPosition());
            telemetry.addLine("-------------------------");

            telemetry.addLine("PIVOTS");
            telemetry.addData("pivot R POW", linearPivotRight.getPower());
            telemetry.addData("pivot L POW", linearPivotLeft.getPower());
            telemetry.addData("pivot R POS", linearPivotRight.getCurrentPosition());
            telemetry.addData("pivot L POS", linearPivotLeft.getCurrentPosition());

            telemetry.update();

        }

    }



    public double calculateSlidePower(double linearSlidePosition) {
        double linearSlidePower = 0;
        double linearSlideCushion = 1;

        // determine the cushion for the linear slide so robot does not exceed extension limit
        linearSlideCushion = (4200.0 - linearSlidePosition)
                /400.0;

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


        return linearSlidePower;
    }

    public void initHardware() {
        linearSlideRight = hardwareMap.get(DcMotorEx.class, "linearSlideRight");
        linearSlideLeft = hardwareMap.get(DcMotorEx.class, "linearSlideLeft");
        linearPivotRight = hardwareMap.get(DcMotorEx.class, "linearPivotRight");
        linearPivotLeft = hardwareMap.get(DcMotorEx.class, "linearPivotLeft");

        linearSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearPivotLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearPivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
