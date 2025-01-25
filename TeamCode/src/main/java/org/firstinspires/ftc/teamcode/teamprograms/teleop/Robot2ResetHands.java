package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "A")
public class Robot2ResetHands extends LinearOpMode {

    CRServo linearActuatorRight, linearActuatorLeft;
    Servo intakePivot;

    @Override
    public void runOpMode() {

        // linearActuatorRight = hardwareMap.get(CRServo.class, "linearActuatorRight");
        // linearActuatorLeft = hardwareMap.get(CRServo.class, "linearActuatorLeft");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        intakePivot.setPosition(0.7);

        telemetry.addLine("Press Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // run actuators
            if (gamepad2.dpad_up) {
                // go up
                linearActuatorRight.setPower(1.0);
                linearActuatorLeft.setPower(1.0);
            } else if (gamepad2.dpad_down) {
                // go down
                linearActuatorRight.setPower(-1.0);
                linearActuatorLeft.setPower(-1.0);
            } else {
                // stay
                linearActuatorRight.setPower(0);
                linearActuatorLeft.setPower(0);
            }

            telemetry.addData("LR POW", linearActuatorRight.getPower());
            telemetry.addData("LL POW", linearActuatorLeft.getPower());
            telemetry.update();

        }

    }
}
