package org.firstinspires.ftc.teamcode.teamprograms.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Penny Pincher", group = "A")
public class Meet0Temp extends LinearOpMode {

    DcMotorEx frontRight, backRight, frontLeft, backLeft;


    @Override
    public void runOpMode() {

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Press Start");
        telemetry.update();
        waitForStart();

        frontRight.setPower(-0.3);
        backRight.setPower(-0.3);
        frontLeft.setPower(-0.3);
        backLeft.setPower(-0.3);

        sleep(5000);

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

        telemetry.addLine("Congratulations, +3 points!");
        telemetry.update();

        sleep(30000);


    }

}
