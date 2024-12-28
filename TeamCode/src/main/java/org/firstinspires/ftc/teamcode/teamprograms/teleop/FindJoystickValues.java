package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(group = "ZZZ")
public class FindJoystickValues extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            telemetry.addData("G2 R Stick Y", (-gamepad2.right_stick_y));
            telemetry.addData("G2 L Stick Y", (-gamepad2.left_stick_y));
            telemetry.update();


        }

    }

}
