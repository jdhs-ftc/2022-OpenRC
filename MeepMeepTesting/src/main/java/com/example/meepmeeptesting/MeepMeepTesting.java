package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(36, -61.5, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.4, 52.4, Math.toRadians(180), Math.toRadians(180), 16.34)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineToSplineHeading(new Pose2d(startPose.getX() + 15, startPose.getY() + 50, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(startPose.getX() + 20, startPose.getY() + 50, Math.toRadians(0)), Math.toRadians(0))
                                /*
                                .splineToSplineHeading(new Pose2d(startPose.getX() + 7, startPose.getY() + 25, Math.toRadians(0)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(startPose.getX() + 24, startPose.getY() + 25), Math.toRadians(0))
                                */

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}