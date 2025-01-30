package org.firstinspires.ftc.teamcode.teamprograms.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * program was for use with the HITEC HSRM9382TH Servo
 **/


@TeleOp(group = "AAA")
@Deprecated
@Disabled
@Config
public class Robot2TestSpecimen extends LinearOpMode {

    // 2 grippers use a y connector
    ServoImplEx specimenGripperR;
    ServoImplEx specimenGripperL;

    // use a custom pwm range to get the full potential of the servos
    PwmControl.PwmRange range = new PwmControl.PwmRange(800, 2200);

    public static class SpecimenValues {
        public double open = 0.5;
        public double close = 0.6;
        public double test = 0.5;
    }

    public static SpecimenValues SPECIMEN_VALUES = new SpecimenValues();


    @Override
    public void runOpMode() {

        // init servo
        specimenGripperR = hardwareMap.get(ServoImplEx.class, "specimenGripperR");
        specimenGripperL = hardwareMap.get(ServoImplEx.class, "specimenGripperL");

        specimenGripperR.setPwmRange(range);
        specimenGripperL.setPwmRange(range);

        telemetry.addLine("Press Start");
        telemetry.addLine("SET UP FOR HITEC HSRM9382TH SERVO");
        telemetry.update();

        waitForStart();

        boolean isGripperOpen = false;
        boolean checkGTwoB = false;

        while (opModeIsActive()) {

            if (!gamepad2.b) checkGTwoB = false;

            if (gamepad2.b && !checkGTwoB) {
                checkGTwoB = true;
                isGripperOpen = !isGripperOpen;
            }

            if (isGripperOpen) {
                specimenGripperR.setPosition(SPECIMEN_VALUES.open);
                specimenGripperL.setPosition(SPECIMEN_VALUES.open);
            } else {
                specimenGripperR.setPosition(SPECIMEN_VALUES.close);
                specimenGripperL.setPosition(SPECIMEN_VALUES.close);
            }

            telemetry.addData("Gripper R pos", specimenGripperR.getPosition());
            telemetry.addData("Gripper L pos", specimenGripperL.getPosition());
            telemetry.addData("Gripper open", isGripperOpen);
            telemetry.addData("PWM RANGE R", specimenGripperR.getPwmRange());
            telemetry.addData("PWM RANGE L", specimenGripperL.getPwmRange());
            telemetry.update();

        }

    }
}
