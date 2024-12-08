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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -61.5, Math.PI/2))
                .lineToYLinearHeading(-35, Math.PI*1.5)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(35, -35))
                .strafeTo(new Vector2d(35, -13))
                .splineToLinearHeading(new Pose2d(new Vector2d( 46, -13), Math.PI/2), Math.PI)
                .strafeTo(new Vector2d(46, -48))
                .strafeTo(new Vector2d(46, -13))
                .strafeTo(new Vector2d(55, -13))
                .strafeTo(new Vector2d(55, -48))
                .strafeTo(new Vector2d(55, -45))
                .splineToConstantHeading(new Vector2d(37, -55), -Math.PI/2)
                .strafeTo(new Vector2d(37, -59))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(12, -35), Math.PI*1.5)
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(37, -55), Math.PI*.5)
                .strafeTo(new Vector2d(37, -59))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(8, -35), Math.PI*1.5)
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(37, -55), Math.PI*.5)
                .strafeTo(new Vector2d(37, -59))
                .strafeToLinearHeading(new Vector2d(6, -35), Math.PI*1.5)
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(45, -59), Math.PI*.5)
                //.splineToLinearHeading(new Pose2d(new Vector2d(25, -40), Math.PI/4), Math.PI/4)
//                .strafeToLinearHeading(new Vector2d(35, -60), Math.PI)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}