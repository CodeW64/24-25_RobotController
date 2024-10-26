package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "Z")
public class findDrivetrainValues extends LinearOpMode {

    DcMotorEx frontRight, backRight, frontLeft, backLeft;

    @Override
    public void runOpMode() {

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.y) {
                frontRight.setPower(0.3);
            } else {
                frontRight.setPower(0);
            }

            if (gamepad1.b) {
                backRight.setPower(0.3);
            } else {
                backRight.setPower(0);
            }

            if (gamepad1.x) {
                frontLeft.setPower(0.3);
            } else {
                frontLeft.setPower(0);
            }

            if (gamepad1.a) {
                backLeft.setPower(0.3);
            } else {
                backLeft.setPower(0);
            }

            telemetry.addLine("Y = FR");
            telemetry.addLine("B = BR");
            telemetry.addLine("X = FL");
            telemetry.addLine("A = BL");
            telemetry.addLine("--------------------------------");
            telemetry.addData("FR POS", frontRight.getCurrentPosition());
            telemetry.addData("FR POW", frontRight.getPower());
            telemetry.addData("BR POS", backRight.getCurrentPosition());
            telemetry.addData("BR POW", backRight.getPower());
            telemetry.addData("FL POS", frontLeft.getCurrentPosition());
            telemetry.addData("FL POW", frontLeft.getPower());
            telemetry.addData("BL POS", backLeft.getCurrentPosition());
            telemetry.addData("BL POW", backLeft.getPower());
            telemetry.update();

        }

    }
}
