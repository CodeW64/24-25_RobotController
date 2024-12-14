package com.example.meepmeepandroid;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepAndroid {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, -66, Math.toRadians(90)))
                /*// specimen
                                .lineToYConstantHeading(-36)

                // sample 1
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-36, -48), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48, -36), Math.toRadians(90))


                // basket 1
                        .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-54, 54, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-54, -54, Math.toRadians(225)), Math.toRadians(225))

                // sample 2
                                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-60, -36, Math.toRadians(90)), Math.toRadians(180))

                // basket 2
                                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-54, -54, Math.toRadians(225)), Math.toRadians(225))

                // hang
                                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-48, -24, Math.toRadians(0)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-24, -12), Math.toRadians(0))*/

                .lineToYConstantHeading(-36)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-36, -48), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48, -36), Math.toRadians(90))
                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-54, 54, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-54, -54, Math.toRadians(225)), Math.toRadians(225))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-60, -36, Math.toRadians(90)), Math.toRadians(180))
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-54, -54, Math.toRadians(225)), Math.toRadians(225))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-48, -24, Math.toRadians(0)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-24, -12), Math.toRadians(0))


//                .splineToSplineHeading(new Pose2d(-48, 36, Math.toRadians(135)), Math.toRadians(90))
////                                .lineToY(48)
//                .splineTo(new Vector2d(-48, 48), Math.toRadians(90))
//                        .setReversed(true)



//                .lineToX(30)
//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}