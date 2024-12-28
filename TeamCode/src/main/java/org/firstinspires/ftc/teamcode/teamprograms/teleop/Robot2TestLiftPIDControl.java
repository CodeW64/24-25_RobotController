package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(group = "AAA")
@Config
public class Robot2TestLiftPIDControl extends LinearOpMode {

    Servo intakePivot;

    DcMotorEx linearPivotRight, linearPivotLeft;

    final int PIVOT_DEPOSIT_POSITION = 1650;
    final int PIVOT_INTAKE_POSITION = 100;


    PIDController pivotController;
    ElapsedTime pidTimer = new ElapsedTime();
    ElapsedTime stateTimer = new ElapsedTime();

    public static class PivotConstants {
        public double kp = 0.005;
        public double ki = 0.0;
        public double kd = 0.0; // 0.0002
        public double gravityFeedForward = 0.002;
        public double speedCap = 1.0;
    }
    public static PivotConstants PIVOT_CONSTANTS = new PivotConstants();


    enum PivotStates {
        INTAKE_ACTIVE, PIVOT_TO_DEPOSIT, DEPOSIT_ACTIVE, PIVOT_TO_INTAKE
    }

    PivotStates pivotState = PivotStates.INTAKE_ACTIVE;


    private final double PIVOT_TICKS_PER_DEGREE = 23.26; // (motor PPR / gear ratio) / 360
    private boolean isRunningPivotToPosition = false;


    @Override
    public void runOpMode() {

        // buttons
        boolean checkGTwoB = true;

        initHardware();

        intakePivot.setPosition(0.52);

        pivotController = new PIDController(PIVOT_CONSTANTS.kp, PIVOT_CONSTANTS.ki, PIVOT_CONSTANTS.kd);
        pivotController.setTolerance(10);

        // for dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Press Start");
        telemetry.update();
        pidTimer.startTime();
        stateTimer.startTime();

        waitForStart();

        // variables for automation
        int linearPivotTargetPosition = PIVOT_INTAKE_POSITION;
        boolean isStateInitialized = false;

        while (opModeIsActive()) {

            if (!gamepad2.b) checkGTwoB = false;

            // gather values
            int linearPivotPosition = (linearPivotRight.getCurrentPosition() + linearPivotLeft.getCurrentPosition()) / 2;


            switch (pivotState) {

                // lift pivot not running to position while intake is active
                case INTAKE_ACTIVE:
                    if (!isStateInitialized) {
                        linearPivotRight.setPower(0);
                        linearPivotLeft.setPower(0);
                        isRunningPivotToPosition = false;

                        isStateInitialized = true;
                    }

                    // EXIT

                    // begin pivot to deposit
                    if (gamepad2.b && !checkGTwoB) {
                        checkGTwoB = true;
                        isStateInitialized = false;
                        linearPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        linearPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        pivotState = PivotStates.PIVOT_TO_DEPOSIT;
                    }

                    break;

                // move lift to deposit position
                // uses run to position
                case PIVOT_TO_DEPOSIT:
                    if (!isStateInitialized) {
                        linearPivotTargetPosition = PIVOT_DEPOSIT_POSITION;
                        isRunningPivotToPosition = true;
                        pidTimer.reset();

                        isStateInitialized = true;
                    }

                    // EXIT

                    // go to deposit active once position has been reached
                    if (Math.abs(linearPivotPosition - PIVOT_DEPOSIT_POSITION) < 10) {
                        isStateInitialized = false;
                        pivotState = PivotStates.DEPOSIT_ACTIVE;
                    }

                    break;

                // lift pivot is running to position while deposit is active to maintain position
                case DEPOSIT_ACTIVE:

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
                        linearPivotTargetPosition = PIVOT_INTAKE_POSITION;
                        pidTimer.reset();
                        stateTimer.reset();

                        isStateInitialized = true;
                    }

                    // EXIT

                    // go to intake active once position has been reached
                    if (Math.abs(linearPivotPosition - PIVOT_INTAKE_POSITION) < 100) {
                        // brace for impact!
                        linearPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        linearPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        linearPivotRight.setPower(0);
                        linearPivotLeft.setPower(0);
                        isRunningPivotToPosition = false;

                        isStateInitialized = false;
                        pivotState = PivotStates.INTAKE_ACTIVE;
                    }
                    break;

            } // end switch statement



            // run position power controller for pivot
            if (isRunningPivotToPosition) {
                runPivotToPositionIndividual(
                        linearPivotRight.getCurrentPosition(),
                        linearPivotLeft.getCurrentPosition(),
                        linearPivotTargetPosition);
            } else {
                linearPivotRight.setPower(0);
                linearPivotLeft.setPower(0);
            }


            // telemetry
            telemetry.addData("Pivot State", pivotState);
            telemetry.addData("Running with PIDF", isRunningPivotToPosition);
            telemetry.addData("TIMER", pidTimer.seconds());

            // graph these in dashboard to aid in PIDF tuning
            telemetry.addData("average pivot position", linearPivotPosition);
            telemetry.addData("pivot target position", linearPivotTargetPosition);

            telemetry.addLine("--------------------------------");
            telemetry.addData("Pivot R POS", linearPivotRight.getCurrentPosition());
            telemetry.addData("Pivot L POS", linearPivotLeft.getCurrentPosition());
            telemetry.addData("Pivot R POW", linearPivotRight.getPower());
            telemetry.addData("Pivot L POW", linearPivotLeft.getPower());
            telemetry.update();

        }
    }


    /**
     * Runs and sets pivot motor powers based on a target position while fighting gravity. <br>
     * <strong>This should be used in place of RUN_TO_POSITION for the chain driven pivots!</strong> <br>
     * (note when not in use, powers should manually be set to 0 to avoid residual motor power.)
     * @param currentPosition current average position of pivot motors
     * @param targetPosition desired end position for lift pivot
     */
    private void runPivotToPosition(int currentPosition, int targetPosition) {
        // update PID variables every loop
        pivotController.setPID(PIVOT_CONSTANTS.kp, PIVOT_CONSTANTS.ki, PIVOT_CONSTANTS.kd);

        // calculate power to apply to pivot
        double pivotPID = pivotController.calculate(currentPosition, targetPosition);
        pivotPID = Range.clip(pivotPID, -1.0, 1.0);

        // find feedforward to fight gravity based on arm being completely horizontal when starting
        double pivotFF = Math.cos(Math.toRadians(currentPosition / PIVOT_TICKS_PER_DEGREE + 1)) * PIVOT_CONSTANTS.gravityFeedForward;

        // perhaps a more useful calculation if position is wanted to be found when arm is vertically down
//            ff = Math.sin(Math.toRadians(armPos / ticks_in_degree + zeroOffset )) * f;

//        double pivotPower = (pivotPID * PIVOT_CONSTANTS.speedCap) + pivotFF;

        // modify raw PID power so robot does not rattle apart since kd was not very effective
        double accelerationFactor = Range.clip((pidTimer.seconds()*10), 0, 1.0);
        double acceleratedPower = Math.min(accelerationFactor*pivotPID, PIVOT_CONSTANTS.speedCap);
        double pivotPower = acceleratedPower + pivotFF;

        // apply power to pivot motors
        linearPivotRight.setPower(pivotPower);
        linearPivotLeft.setPower(pivotPower);
    }


    private void runPivotToPositionIndividual(int currentPositionR, int currentPositionL, int targetPosition) {
        // update PID variables every loop
        pivotController.setPID(PIVOT_CONSTANTS.kp, PIVOT_CONSTANTS.ki, PIVOT_CONSTANTS.kd);

        // calculate power to apply to pivots
        double pivotPIDR = pivotController.calculate(currentPositionR, targetPosition);
        pivotPIDR = Range.clip(pivotPIDR, -1.0, 1.0);

        double pivotPIDL = pivotController.calculate(currentPositionL, targetPosition);
        pivotPIDL = Range.clip(pivotPIDL, -1.0, 1.0);

        // find feedforward to fight gravity based on arm being completely horizontal when starting
        double pivotFFR = Math.cos(Math.toRadians(currentPositionR / PIVOT_TICKS_PER_DEGREE + 1)) * PIVOT_CONSTANTS.gravityFeedForward;
        double pivotFFL = Math.cos(Math.toRadians(currentPositionL / PIVOT_TICKS_PER_DEGREE + 1)) * PIVOT_CONSTANTS.gravityFeedForward;

        // perhaps a more useful calculation if position is wanted to be found when arm is vertically down
//            ff = Math.sin(Math.toRadians(armPos / ticks_in_degree + zeroOffset )) * f;

//        double pivotPower = (pivotPID * PIVOT_CONSTANTS.speedCap) + pivotFF;

        // modify raw PID power so robot does not rattle apart since kd was not very effective
        double accelerationFactor = Range.clip((pidTimer.seconds()*10), 0, 1.0);
        double acceleratedPowerR = Math.min(accelerationFactor*pivotPIDR, PIVOT_CONSTANTS.speedCap);
        double acceleratedPowerL = Math.min(accelerationFactor*pivotPIDL, PIVOT_CONSTANTS.speedCap);
        double pivotPowerR = acceleratedPowerR + pivotFFR;
        double pivotPowerL = acceleratedPowerL + pivotFFL;

        // apply power to pivot motors
        linearPivotRight.setPower(pivotPowerR);
        linearPivotLeft.setPower(pivotPowerL);
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


    }
}
