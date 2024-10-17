package org.firstinspires.ftc.teamcode.teamprograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorPivotTest extends LinearOpMode {

    DcMotorEx twistMotor;

    @Override
    public void runOpMode() {

        twistMotor = hardwareMap.get(DcMotorEx.class, "frontRight");

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            double twistMotorPower = gamepad1.right_stick_y;
            twistMotor.setPower(twistMotorPower);

            telemetry.addData("PIVOT POWER", twistMotorPower);
            telemetry.update();



        }


    }

}
