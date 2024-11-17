package org.firstinspires.ftc.teamcode.teamprograms.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotLift;


/*
 * NOTES:
 * 0,0 is at the center of the field in the jigsaw crossing
 * Coordinates are based on red alliance side as base with x parallel and y perpendicular
 * Each field tile is 24 inches
 * Coordinates for start position and movement are estimations and based on red alliance
 */
@Autonomous(name = "Steve's Wackiness Test", group = "Autotonomice")
public class BasketPathTestSingleThread extends LinearOpMode {

    MecanumDrive drive;
    TelemetryDrive telemetryDrive;
    RobotLift lift;

    Pose2d startPose = new Pose2d(-12, -66, Math.toRadians(90)); // estimation

    private final double TAG_DISTANCE = 0;
    private final double TAG_BEARING = 0;
    private final double TAG_YAW = 0;
    private final double TAG_ANGLE = Math.toRadians(0);


    public void runOpMode() {

        drive = new MecanumDrive(hardwareMap, startPose);
        telemetryDrive = new TelemetryDrive();
        lift = new RobotLift(hardwareMap, telemetry);


        // set up velocity and acceleration constraints to be used later
        TranslationalVelConstraint velocitySlow = new TranslationalVelConstraint(25);
        ProfileAccelConstraint accelerationSlow = new ProfileAccelConstraint(-25, 25);

        // roadrunner 1.0 is quite different, first up is creation of trajectories

        // move to hang specimen
        TrajectoryActionBuilder trajectory1 = drive.actionBuilder(startPose)
                .lineToYConstantHeading(-36);

        // move to sample spike mark 1 (from specimen position)
        TrajectoryActionBuilder trajectory2 = trajectory1.fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-36, -48), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48, -36), Math.toRadians(90));

        // move to basket (from spike mark 1 position)
        TrajectoryActionBuilder trajectory3 = trajectory2.fresh()
                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-54, 54, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-54, -54, Math.toRadians(225)), Math.toRadians(225),
                        velocitySlow, accelerationSlow);

        // move to sample spike mark 2 (from basket)
        TrajectoryActionBuilder trajectory4 = trajectory3.fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-60, -36, Math.toRadians(90)), Math.toRadians(180));

        // move to basket (from spike mark 2 position)
        TrajectoryActionBuilder trajectory5 = trajectory4.fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-54, -54, Math.toRadians(225)), Math.toRadians(225),
                        velocitySlow, accelerationSlow);

        // go get a level 1 ascent (from basket)
        TrajectoryActionBuilder trajectory6 = trajectory5.fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-48, -24, Math.toRadians(0)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-24, -12), Math.toRadians(0));




        // last is turning the trajectories into actions
        // key: b is for build (not Bob the Builder)

        Action bStartToSpecimen = trajectory1.build();
        Action bSpecimenToSample1 = trajectory2.build();
        Action bSample1ToBasket1 = trajectory3.build();
        Action bBasket1ToSample2 = trajectory4.build();
        Action bSample2ToBasket2 = trajectory5.build();
        Action bBasket2ToAscent = trajectory6.build();

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(lift.duck.spinDuck());


        // run the autonomous
        Actions.runBlocking(new SequentialAction(
                bStartToSpecimen,
                telemetryDrive.displayPoseAction("Specimen End"),
                bSpecimenToSample1,
                telemetryDrive.displayPoseAction("Sample 1 End")
        ));

        // check for sample grab here
        // if not, move over a little and re-grab

        Actions.runBlocking(new SequentialAction(
                bSample1ToBasket1,
                telemetryDrive.displayPoseAction("Basket 1 End"),

                // runs AprilTag
                /*new ParallelAction(
                        new SequentialAction(
                                // some april tag action
                        ),
                        new SequentialAction(
                                // some lift action
                        )
                ),*/

                bBasket1ToSample2,
                telemetryDrive.displayPoseAction("Sample 2 End")
        ));


        // check for sample grab here
        // if not, move over a little and re-grab

        Actions.runBlocking(new SequentialAction(
                bSample2ToBasket2,
                telemetryDrive.displayPoseAction("Basket 2 End"),
                bBasket2ToAscent,
                telemetryDrive.displayPoseAction("Ascent End")
        ));

        // end the autonomous
        sleep(30000);



    }


    private class TelemetryDrive {

        private Action displayPoseAction(String caption) {
            return new Action() {
                private final String givenCaption = caption;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    drive.updatePoseEstimate();
                    telemetry.addLine(caption);
                    telemetry.addData("x", drive.pose.position.x);
                    telemetry.addData("y", drive.pose.position.y);
                    telemetry.addData("Θ", drive.pose.heading.toDouble());
                    telemetry.addLine("--------------------------------");
                    telemetry.update();
                    return false;
                }
            };
        }


    }


}
