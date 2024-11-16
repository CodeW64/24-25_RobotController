package org.firstinspires.ftc.teamcode.teamprograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

@TeleOp(group = "Quack")
public class DuckLED extends LinearOpMode {

    LED duckLEDRed, duckLEDGreen;

    @Override
    public void runOpMode() {

        duckLEDRed = hardwareMap.get(LED.class, "duckLEDRed");
        duckLEDGreen = hardwareMap.get(LED.class, "duckLEDGreen");

        telemetry.addLine("Press Start to enable light");
        telemetry.update();
        waitForStart();

        duckLEDRed.enable(true);
        duckLEDGreen.enable(false);

        while (opModeIsActive()) {

            // enable or disable red LED
            if (gamepad1.b) {
                duckLEDRed.enable(true);
            } else if (gamepad1.a) {
                duckLEDRed.enable(false);
            }

            // enable or disable green LED
            if (gamepad1.y) {
                duckLEDGreen.enable(true);
            } else if (gamepad1.x) {
                duckLEDGreen.enable(false);
            }

            telemetry.addLine("B: enable red LED");
            telemetry.addLine("A: disable red LED");
            telemetry.addLine("Y: enable green LED");
            telemetry.addLine("X: disable green LED");
            telemetry.addLine("--------------------");
            telemetry.addData("Red LED", duckLEDRed);
            telemetry.addData("Green LED", duckLEDGreen);
            telemetry.update();
        }


    }
}
