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
    final static VelConstraint slowVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(30),
                    new AngularVelConstraint(Math.PI)
            ));

    public static double intakeWaitTime = 2.0;
    public static double outtakeWaitTime = 2.0;
    public static double specimenWaitTime = 0.5;
    public static double specimenIntakeTime = 0.1;
    public static double specimenOuttakeTime = 0.5;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 15.984252)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -61.5, -Math.PI/2))
                //sample auto pathing
/*                .strafeTo(new Vector2d(-10, -35))
                .waitSeconds(specimenWaitTime)
                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(80)), Math.PI)
                .waitSeconds(intakeWaitTime)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.PI)
                .waitSeconds(outtakeWaitTime)
                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(105)), Math.PI)
                .waitSeconds(intakeWaitTime)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.PI)
                .waitSeconds(outtakeWaitTime)
                .splineToLinearHeading(new Pose2d(-53, -51, Math.toRadians(120)), Math.PI)
                .waitSeconds(intakeWaitTime)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.PI)
                .waitSeconds(outtakeWaitTime)
                .splineToLinearHeading(new Pose2d(-24, -10, Math.PI), Math.toRadians(0)) */
                //specimen auto pathing
                .strafeTo(new Vector2d(10, -35))
                .waitSeconds(specimenOuttakeTime)
                .splineToLinearHeading(new Pose2d(20, -45, Math.toRadians(45)), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(25, -50, Math.toRadians(-30)), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(30, -45, Math.toRadians(45)), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(35, -50, Math.toRadians(-30)), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(45)), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(35, -50, Math.toRadians(-30)), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(37, -55, Math.toRadians(90)), Math.toRadians(-90))
                .strafeTo(new Vector2d(37, -60), slowVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeToSplineHeading(new Vector2d(13, -35), -Math.PI/2)
                .waitSeconds(specimenOuttakeTime)
                .strafeToSplineHeading(new Vector2d(37, -55), Math.PI/2)
                .strafeTo(new Vector2d(37, -60), slowVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeToSplineHeading(new Vector2d(7, -35), -Math.PI/2)
                .waitSeconds(specimenOuttakeTime)
                .strafeToSplineHeading(new Vector2d(37, -55), Math.PI/2)
                .strafeTo(new Vector2d(37, -60), slowVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeToSplineHeading(new Vector2d(4, -35), -Math.PI/2)

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