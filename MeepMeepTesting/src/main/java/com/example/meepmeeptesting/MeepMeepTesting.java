package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {

    final static MecanumKinematics kinematics = new MecanumKinematics(
            15.984252, 0.8);
    final static VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(10),
                    new AngularVelConstraint(Math.PI)
            ));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 15.984252)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -61.5, Math.PI/2))
                .strafeTo(new Vector2d(10, -35))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(35, -35))
                .strafeTo(new Vector2d(35, -13))
                .splineToLinearHeading(new Pose2d(new Vector2d( 48, -13), 0), -Math.PI/2)
                .strafeTo(new Vector2d(48, -48))
                .strafeTo(new Vector2d(48, -13))
                .strafeTo(new Vector2d(57, -13))
                .strafeTo(new Vector2d(57, -48))
                .strafeTo(new Vector2d(57, -47))
                .splineToConstantHeading(new Vector2d(37, -55), -Math.PI/2)
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .waitSeconds(0.1)
                .strafeToSplineHeading(new Vector2d(13, -35), Math.PI)
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(37, -55), 0)
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .waitSeconds(0.1)
                .strafeToSplineHeading(new Vector2d(7, -35), Math.PI)
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(37, -55), 0)
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .strafeToSplineHeading(new Vector2d(4, -35), Math.PI)

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