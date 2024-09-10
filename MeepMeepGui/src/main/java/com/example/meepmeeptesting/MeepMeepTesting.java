package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

                // -10.298715017559747, -19.179107030802854
        final double x = -10.298715017559747;
        final double y = -19.179107030802854;
        final double turn = -138.74903012168764;
        // final double turnRad = Math.toRadians(turn);
        final double turnRad = 3.321970851313089;

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Vector2d(x, y), turnRad)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}