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
                .setConstraints(65, 65, Math.toRadians(180), Math.toRadians(180), 12.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(7, -61.5, -Math.PI/2))
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
                .waitSeconds(0.35)
                .strafeTo(new Vector2d(1, -35))
                .waitSeconds(specimenOuttakeTime)
                /*
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(29.14, -34.63), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(39.97, -13.72), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(45.90, -23.80), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(45.75, -54.00), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(50.50, -13.42), Math.toRadians(12.00))
                .splineToConstantHeading(new Vector2d(55.00, -19.95), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(55.00, -54.00), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(58.65, -12.83), Math.toRadians(10.23))
                .splineToConstantHeading(new Vector2d(63.50, -19.65), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(63.50, -54.00), Math.toRadians(-90))
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(37, -55), -Math.PI/2)
                 */
                .splineToLinearHeading(new Pose2d(27.00, -43.00, Math.toRadians(225)), Math.toRadians(-25.54))
                .splineToLinearHeading(new Pose2d(33.33, -45.16, Math.toRadians(135.00)), Math.toRadians(-18.18))
                .splineToLinearHeading(new Pose2d(35.00, -43.00, Math.toRadians(225)), Math.toRadians(-25.54))
                .splineToLinearHeading(new Pose2d(37.33, -45.16, Math.toRadians(135.00)), Math.toRadians(-18.18))
                .splineToLinearHeading(new Pose2d(40.00, -43.00, Math.toRadians(225)), Math.toRadians(-25.54))
                .splineToLinearHeading(new Pose2d(42.33, -45.16, Math.toRadians(135.00)), Math.toRadians(-18.18))
                .strafeTo(new Vector2d(37, -60), slowVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(13, -35))
                .waitSeconds(specimenOuttakeTime)
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), slowVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(10, -35))
                .waitSeconds(specimenOuttakeTime)
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), slowVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(7, -35))
                .waitSeconds(specimenOuttakeTime)
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), slowVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(4, -35))
                .waitSeconds(specimenOuttakeTime)
                .strafeToConstantHeading(new Vector2d(45, -59))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}