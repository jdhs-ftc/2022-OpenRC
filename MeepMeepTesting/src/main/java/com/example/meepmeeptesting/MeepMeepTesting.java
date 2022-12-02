package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-36, -61.5, Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setDimensions(14,17)
                .setConstraints(52.48, 52.48, Math.toRadians(266.7532763442901), Math.toRadians(184.02607784577722), 11.31)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)

                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(25))
                                .strafeRight(12)
                                .splineToSplineHeading(new Pose2d(-12, -24, Math.toRadians(0)), Math.toRadians(90))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10,Math.toRadians(266.753),11.31))
                                .forward(6)
                                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                                    //motorControl.setMode(motorControl.combinedMode.TOP);
                                })
                                .addTemporalMarker(() -> {
                                    //motorControl.setMode(motorControl.combinedMode.MIDDLE);
                                })
                                .waitSeconds(1)
                                .back(6)
                                .resetVelConstraint()
                                .strafeRight(12)
                                .addDisplacementMarker(() -> {

                                })
                                .resetAccelConstraint()

                                // to cone stack
                                /*
                                .splineToSplineHeading(new Pose2d(startPose.getX() + 15, startPose.getY() + 50, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(startPose.getX() + 20, startPose.getY() + 50, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(62, -11.5, Math.toRadians(0)), Math.toRadians(0))
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