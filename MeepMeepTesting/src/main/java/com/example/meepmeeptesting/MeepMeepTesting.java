package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(38, 61.5, -Math.PI/2))
                .splineTo(new Vector2d(54.5, 54.5), Math.PI/4)
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(58, 45), Math.PI/2)
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(54.5, 54.5), Math.PI/4)
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(52, 45), Math.toRadians(75))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(54.5, 54.5), Math.PI/4)
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(34, 10), 0)
//                .strafeToLinearHeading(new Vector2d(35, -60), Math.PI)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}