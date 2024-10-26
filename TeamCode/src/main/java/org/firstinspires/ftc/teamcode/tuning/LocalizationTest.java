package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.gobilda.RRGobildaLocalizer;

public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            telemetry.setMsTransmissionInterval(33);

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -(gamepad1.left_stick_y),
                                -(gamepad1.left_stick_x)
                        ),
                        -(gamepad1.right_stick_x)
                ));

                drive.updatePoseEstimate();
                Pose2D bildaPos = drive.bildaDriver.getPosition();
                Pose2D bildaVel = drive.bildaDriver.getVelocity();



                telemetry.addData("goBilda IMU direct heading (DEG)", bildaPos.getHeading(AngleUnit.DEGREES));
                telemetry.addData("goBilda IMU direct heading (RAD)", bildaPos.getHeading(AngleUnit.RADIANS));
                telemetry.addData("goBilda IMU direct velocity (DEG)", bildaVel.getHeading(AngleUnit.DEGREES));
                telemetry.addData("goBilda IMU direct velocity (RAD)", bildaVel.getHeading(AngleUnit.RADIANS));
                if (drive.localizer instanceof RRGobildaLocalizer) {
                    telemetry.addData("goBilda IMU hasReturnedNaN", ((RRGobildaLocalizer) drive.localizer).hasReturnedNaN);
                }
                telemetry.addData("Drive Pose hasReturnedNaN", drive.hasReturnedNaN);
                telemetry.addLine("--------------------------------");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.addData("heading (toDouble)", drive.pose.heading.toDouble());
                telemetry.addLine("--------------------------------");

                // this value is equal to the yaw
                telemetry.addData("LOCALIZER heading (radians)", drive.pose.heading.toDouble());
                telemetry.addData("CLIPPED heading (radians)", Range.clip(drive.pose.heading.toDouble(), -3.1415, 3.1415));
                telemetry.addLine("--------------------------------");

                if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                    telemetry.addData("Turn Count", ((TwoDeadWheelLocalizer) drive.localizer).turnCount);
                }

                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
