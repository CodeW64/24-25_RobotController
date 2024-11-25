package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "Z")
@Config
public class FindLevel3Ascent extends LinearOpMode {

    DcMotorEx linearSlideLift, linearSlidePivot;
    DcMotorEx linearActuatorRight, linearActuatorLeft;
    DcMotorEx frontRight, backRight, frontLeft, backLeft;

    private final double SLIDE_EXTENSION_MAX = 4200;
    private final double CUSHION_RATIO = 400;
//    private final double GRAVITY_COEFFICIENT = 0.0005;

    public static class gravity {
        public double coefficient = 0.0005;
    }
    public static gravity GRAVITY = new gravity();


    private final double ACTUATOR_SPEED = 1.0;
    private final double SLIDE_SPEED = 0.7;
    private final double PIVOT_SPEED = 0.4;

    private final double DRIVE_SPEED = 0.6;

    private boolean enableGravity = true;


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

//        linearActuatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        linearActuatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean checkGTwoA = true;


        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (!gamepad2.a) checkGTwoA = false;

            // enable/disable gravity
            if (gamepad2.a && !checkGTwoA) {
                checkGTwoA = true;
                enableGravity = !enableGravity;
            }

            // get positions
            double linearSlidePosition = linearSlideLift.getCurrentPosition();


            // move linear actuators
            double powerActuatorUp = (-gamepad2.right_trigger) * ACTUATOR_SPEED;
            double powerActuatorDown = (-gamepad2.left_trigger) * ACTUATOR_SPEED;
            linearActuatorRight.setPower(powerActuatorUp - powerActuatorDown);
            linearActuatorLeft.setPower(powerActuatorUp - powerActuatorDown);

            // move linear slide
            double powerSlide = calculateSlidePower(linearSlidePosition);
            linearSlideLift.setPower(powerSlide);

            // move pivot
            double powerPivot = (-gamepad2.left_stick_y) * PIVOT_SPEED;
            linearSlidePivot.setPower(powerPivot);



            // tank mecanum drive

            double rightPower = -gamepad1.right_stick_y * DRIVE_SPEED;
            double leftPower = -gamepad1.left_stick_y * DRIVE_SPEED;

            double slideToRight = -gamepad1.right_trigger * DRIVE_SPEED * 1.1;
            double slideToLeft = -gamepad1.left_trigger * DRIVE_SPEED * 1.1;

            double tankDenominator = Math.max(Math.abs((rightPower+leftPower)/2) + Math.abs(slideToRight) + Math.abs(slideToLeft), 1);

            frontRight.setPower((rightPower + slideToRight - slideToLeft) / tankDenominator);
            backRight.setPower((rightPower - slideToRight + slideToLeft) / tankDenominator);
            frontLeft.setPower((leftPower - slideToRight + slideToLeft)/ tankDenominator);
            backLeft.setPower((leftPower + slideToRight - slideToLeft)/ tankDenominator);



            telemetry.addData("GRAVITY COEFFICIENT ENABLED", enableGravity);
            telemetry.addLine("G2 button_a enables/disables gravity");
            telemetry.addLine("NOTE if gravity is on all the time, the motor will burn");
            telemetry.addLine("--------------------------------");
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

        // use the power cushion if robot is extending, ignore if retracting
        if ((-gamepad2.right_stick_y) > 0) {
            linearSlidePower = (((-gamepad2.right_stick_y)*SLIDE_SPEED*0.75)*
                    linearSlideCushion);
        } else {
            // make lowering slides slower
            linearSlidePower = ((-gamepad2.right_stick_y)*SLIDE_SPEED*0.5);
        }

        // make descent of slides slightly slower so it is not jarring
        /*if ((-gamepad2.right_stick_y) < 0) {
            linearSlidePower*=0.9;
        }*/


        // apply a coefficient to fight gravity
        if (enableGravity) {
            linearSlidePower-=GRAVITY.coefficient;
        }

        return linearSlidePower;
    }

}
