package org.firstinspires.ftc.teamcode.teamprograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorPivotTest extends LinearOpMode {

    DcMotorEx twistMotor, otherTwistMotor;

    @Override
    public void runOpMode() {

        twistMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        otherTwistMotor = hardwareMap.get(DcMotorEx.class, "backRight");


        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            double twistMotorPower = gamepad1.right_stick_y;
            if (gamepad1.a) {
                twistMotorPower = 0.05;
            }
            twistMotor.setPower(twistMotorPower);
            otherTwistMotor.setPower(twistMotorPower);



            telemetry.addData("PIVOT POWER", twistMotorPower);
            telemetry.addData("OTHER PIVOT POWER", twistMotorPower);
            telemetry.update();



        }


    }

}
