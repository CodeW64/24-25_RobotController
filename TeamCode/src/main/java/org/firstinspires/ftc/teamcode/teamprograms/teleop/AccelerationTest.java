package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "Z")
public class AccelerationTest extends LinearOpMode  {

    DcMotorEx linearSlidePivot;

    ElapsedTime lightTimer;

    boolean isInitialized = false;

    @Override
    public void runOpMode() {

        linearSlidePivot = hardwareMap.get(DcMotorEx.class, "linearSlidePivot");
        linearSlidePivot.setDirection(DcMotorSimple.Direction.REVERSE);

        lightTimer = new ElapsedTime();

        lightTimer.startTime();

        while ((opModeInInit())) {
            telemetry.addData("TIME:", lightTimer.seconds());
            telemetry.addLine("Press Start");
            telemetry.update();
        }

        while (opModeIsActive()) {

            if (!isInitialized) {
                lightTimer.reset();
                linearSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linearSlidePivot.setTargetPosition(2400);
                linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlidePivot.setPower(0);
                isInitialized = true;
            }

//            double timeAccel = Math.min((lightTimer.seconds()*2), 0.9);
//            linearSlidePivot.setPower(timeAccel);

            linearSlidePivot.setPower(1.0);


            telemetry.addData("TIME:", lightTimer.seconds());
            telemetry.addData("POWER", linearSlidePivot.getPower());
            telemetry.addData("POSITION", linearSlidePivot.getCurrentPosition());
            telemetry.update();
        }

    }
}
