package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(group = "AAA")
public class FindResetAndSpecimenValues extends LinearOpMode {

    DcMotorEx linearSlideLift, linearSlidePivot;
    TouchSensor linearSlideSwitch, pivotResetSwitch;
    Servo intakePivot;

    final int PIVOT_SPECIMEN_POSITION = 2300;

    enum SpecimenStates {
        PIVOT_STATIONARY, RESET_PIVOT_LOWER, RESET_PIVOT_LEVEL, PIVOT_TO_SPECIMEN
    }

    SpecimenStates specimenState = SpecimenStates.PIVOT_STATIONARY;

    public void runOpMode() {

        linearSlideLift = hardwareMap.get(DcMotorEx.class, "linearSlideLift");
        linearSlidePivot = hardwareMap.get(DcMotorEx.class, "linearSlidePivot");

        linearSlideLift.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlidePivot.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearSlideSwitch = hardwareMap.get(TouchSensor.class, "linearSlideSwitch");
        pivotResetSwitch = hardwareMap.get(TouchSensor.class, "pivotResetSwitch");

        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        boolean isStateInitialized = false;
        boolean isArmPositionSet = false;

        boolean checkGOneDRIGHT = false;
        boolean checkGOneX = false;
        boolean checkGOneB = false;

        intakePivot.setPosition(0.52);

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (!gamepad1.dpad_right) checkGOneDRIGHT = false;
            if (!gamepad1.x) checkGOneX = false;
            if (!gamepad1.b) checkGOneB = false;

            boolean limitSwitch = linearSlideSwitch.isPressed();
            boolean resetSwitch = pivotResetSwitch.isPressed();

            int pivotPosition = linearSlidePivot.getCurrentPosition();



            switch (specimenState) {

                case PIVOT_STATIONARY:


                    if (gamepad1.dpad_right && !checkGOneDRIGHT && gamepad1.x && !checkGOneX) {
                        checkGOneDRIGHT = true;
                        checkGOneX = true;
                        isStateInitialized = false;
                        specimenState = SpecimenStates.RESET_PIVOT_LOWER;
                    }

                    if (gamepad1.b && !checkGOneB) {
                        checkGOneB = true;
                        isStateInitialized = false;
                        specimenState = SpecimenStates.PIVOT_TO_SPECIMEN;
                    }

                    break;


                case PIVOT_TO_SPECIMEN:

                    if (!isStateInitialized) {
                        linearSlidePivot.setTargetPosition(PIVOT_SPECIMEN_POSITION);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(0.5);
                        isStateInitialized = true;
                    }

                    if (Math.abs(pivotPosition - PIVOT_SPECIMEN_POSITION) < 10) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        specimenState = SpecimenStates.PIVOT_STATIONARY;
                    }

                    break;

                case RESET_PIVOT_LOWER:

                    if (!isStateInitialized) {
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlidePivot.setPower(-0.3);
                        isStateInitialized = true;
                    }

                    // fully retract slides
                    if (!limitSwitch) {
                        linearSlideLift.setPower(-0.8);
                    } else {
                        linearSlideLift.setPower(0);
                    }


                    // EXIT/ABORT

                    // leave once pivot hits the switch (or told to exit manually)
                    // reset encoder position so pivot knows where it is and go to level mode
                    if (resetSwitch ||
                            (gamepad1.dpad_right && !checkGOneDRIGHT && gamepad1.x && !checkGOneX)) {
                        checkGOneDRIGHT = true;
                        checkGOneX = true;
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isStateInitialized = false;
                        specimenState = SpecimenStates.RESET_PIVOT_LEVEL;
                    }
                    break;

                // move pivot back to 0 once reset switch has been hit
                // (used in case robot disconnects)
                case RESET_PIVOT_LEVEL:

                    if (!isStateInitialized) {
                        linearSlidePivot.setTargetPositionTolerance(3);
                        linearSlidePivot.setTargetPosition(70);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlidePivot.setPower(0.9);
                        isArmPositionSet = false;
                        isStateInitialized = true;
                    }

                    // in case slide did not have time to fully retract
                    if (isLinearSlideFullyRetracted(limitSwitch) && !isArmPositionSet) {
                        linearSlideLift.setPower(0);
                        linearSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        isArmPositionSet = true;
                    }


                    // EXIT

                    // go back to INTAKE_ACTIVE (normal functions) once arm has finished resetting
                    if (Math.abs(pivotPosition - 70) < 5 &&
                            isArmPositionSet) {
                        linearSlidePivot.setPower(0);
                        linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linearSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlidePivot.setTargetPositionTolerance(5); // default
                        isStateInitialized = false;
                        specimenState = SpecimenStates.PIVOT_STATIONARY;
                    }
                    break;
            }



            telemetry.addData("LIFT POS", linearSlideLift.getCurrentPosition());
            telemetry.addData("PIVOT POS", pivotPosition);
            telemetry.update();
        }

    }

    public boolean isLinearSlideFullyRetracted(boolean limitSwitch) {
        return limitSwitch || gamepad2.x || gamepad2.y;
    }

}
