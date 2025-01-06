package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Lift should start in intake position for opmode to function correctly
 * (Note that slides should be fully retracted and not move while testing)
 * (Also note that slides must be fully retracted to pivot since motors are not strong enough)
 **/

@TeleOp(group = "ZZZ")
@Deprecated
public class Robot2TestRunToPosition extends LinearOpMode {

    Servo intakePivot;

    DcMotorEx linearPivotRight, linearPivotLeft;


    final int PIVOT_DEPOSIT_POSITION = 1650;
    final int PIVOT_INTAKE_POSITION = 100;

    enum PivotStates {
        INTAKE_ACTIVE, PIVOT_TO_DEPOSIT, DEPOSIT_ACTIVE, PIVOT_TO_INTAKE
    }

    PivotStates pivotState = PivotStates.INTAKE_ACTIVE;

    @Override
    public void runOpMode() {

        boolean checkGTwoB = true;

        initHardware();

        intakePivot.setPosition(0.52);

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        boolean isStateInitialized = false;

        while (opModeIsActive()) {

            if (!gamepad2.b) checkGTwoB = false;


            // gather values
            int pivotPosition = (linearPivotRight.getCurrentPosition() + linearPivotLeft.getCurrentPosition()) / 2;


            switch (pivotState) {

            // lift pivot not running to position while intake is active
                case INTAKE_ACTIVE:
                    if (!isStateInitialized) {
                        linearPivotRight.setPower(0);
                        linearPivotLeft.setPower(0);
                        linearPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        isStateInitialized = true;
                    }

                    // EXIT

                    // begin pivot to deposit
                    if (gamepad2.b && !checkGTwoB) {
                        checkGTwoB = true;
                        isStateInitialized = false;
                        pivotState = PivotStates.PIVOT_TO_DEPOSIT;
                    }


                    break;

            // move lift to deposit position
            // uses run to position
                case PIVOT_TO_DEPOSIT:
                    if (!isStateInitialized) {
                        linearPivotRight.setTargetPosition(PIVOT_DEPOSIT_POSITION);
                        linearPivotLeft.setTargetPosition(PIVOT_DEPOSIT_POSITION);
                        linearPivotRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearPivotLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearPivotRight.setPower(1.0);
                        linearPivotLeft.setPower(1.0);

                        isStateInitialized = true;
                    }

                    // EXIT

                    // go to deposit active once position has been reached
                    if (Math.abs(pivotPosition - PIVOT_DEPOSIT_POSITION) < 10) {
                        isStateInitialized = false;
                        pivotState = PivotStates.DEPOSIT_ACTIVE;
                    }

                    break;

            // lift pivot is running to position while deposit is active to maintain position
                case DEPOSIT_ACTIVE:
                    if (!isStateInitialized) {
                        linearPivotRight.setPower(0.5);
                        linearPivotLeft.setPower(0.5);
                        isStateInitialized = true;
                    }

                    // EXIT

                    // begin pivot to intake
                    if (gamepad2.b && !checkGTwoB) {
                        checkGTwoB = true;
                        isStateInitialized = false;
                        pivotState = PivotStates.PIVOT_TO_INTAKE;
                    }
                    break;

            // move lift to intake position
            // uses run to position
                case  PIVOT_TO_INTAKE:
                    if (!isStateInitialized) {
                        linearPivotRight.setPower(-0.7);
                        linearPivotLeft.setPower(-0.7);
                        linearPivotRight.setTargetPosition(PIVOT_INTAKE_POSITION);
                        linearPivotLeft.setTargetPosition(PIVOT_INTAKE_POSITION);

                        isStateInitialized = true;
                    }

                    // EXIT

                    // go to intake active once position has been reached
                    if (Math.abs(pivotPosition - PIVOT_INTAKE_POSITION) < 10) {
                        isStateInitialized = false;
                        pivotState = PivotStates.INTAKE_ACTIVE;
                    }
                    break;

            } // end switch statement


            telemetry.addData("Pivot State", pivotState);
            telemetry.addData("average pivot position", pivotPosition);
            telemetry.addLine("--------------------------------");
            telemetry.addData("Pivot R POS", linearPivotRight.getCurrentPosition());
            telemetry.addData("Pivot L POS", linearPivotLeft.getCurrentPosition());
            telemetry.addData("Pivot R POW", linearPivotRight.getPower());
            telemetry.addData("Pivot L POW", linearPivotLeft.getPower());
            telemetry.update();
        }
    }



    private void initHardware() {

        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        linearPivotRight = hardwareMap.get(DcMotorEx.class, "linearPivotRight");
        linearPivotLeft = hardwareMap.get(DcMotorEx.class, "linearPivotLeft");

        linearPivotRight.setDirection(DcMotorSimple.Direction.REVERSE);

        linearPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearPivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*linearPivotRight.setPower(0);
        linearPivotLeft.setPower(0);*/

    }
}
