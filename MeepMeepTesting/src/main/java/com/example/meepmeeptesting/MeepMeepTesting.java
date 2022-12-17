package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(36, -62, Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setDimensions(14,17)
                .setConstraints(52.48, 52.48, Math.toRadians(266.7532763442901), Math.toRadians(184.02607784577722), 11.31)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                /*
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> motorControl.claw.setPower(0.5))
                                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> motorControl.slide.setTargetPosition(400))

                                 */
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(15))
                                // place cone on small
                                .strafeLeft(12)
                                .forward(4)
                                /*
                                .UNSTABLE_addTemporalMarkerOffset(-3, () -> motorControl.setMode(motorControl.combinedMode.TOP))

                                .UNSTABLE_addTemporalMarkerOffset(-0.1,() -> motorControl.slide.setTargetPosition(motorControl.slide.getTargetPosition()-300))
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> motorControl.claw.setPower(-0.75))
                                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> motorControl.setMode(motorControl.combinedMode.TOP))
                                */

                                .waitSeconds(1.5)
                                .back(4)
                                .waitSeconds(0.1)
                                // go to cone stack
                                .strafeLeft(0.5)
                                .splineToConstantHeading(new Vector2d(12,-48), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(12,-24, Math.toRadians(0)), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(56,-12), Math.toRadians(0))
                                .waitSeconds(0.1)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10,Math.toRadians(10),11.31))
                                .turn(Math.toRadians(180))
                                .resetVelConstraint()
                                //TODO: pick up cone

                                .splineToConstantHeading(new Vector2d(12,-23.9), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(12, -24), Math.toRadians(-90))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10,Math.toRadians(266.753),11.31))
                                .forward(0.5)


                                /*
                                .UNSTABLE_addTemporalMarkerOffset(-3, () -> motorControl.setMode(motorControl.combinedMode.TOP))

                                .UNSTABLE_addTemporalMarkerOffset(-0.1,() -> motorControl.slide.setTargetPosition(motorControl.slide.getTargetPosition()-300))
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> motorControl.claw.setPower(-0.75))
                                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> motorControl.setMode(motorControl.combinedMode.TOP))

                                 */


                                .waitSeconds(1.5)
                                .back(2.5)
                                .resetVelConstraint()
                                .strafeLeft(12)
                                /*
                                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> motorControl.claw.setPower(0.25))
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    motorControl.setMode(motorControl.combinedMode.BOTTOM);
                                    if (tagOfInterest != null) {
                                        if (tagOfInterest.id == 2) {
                                            // TODO: drive to 1 position
                                            TrajectorySequence backFar = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                    .back(47)
                                                    .build();
                                            drive.followTrajectorySequenceAsync(backFar);

                                        } else if (tagOfInterest.id == 1) {
                                            // TODO: drive to 2 position

                                            TrajectorySequence backMiddle = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                    .back(24)
                                                    .build();
                                            drive.followTrajectorySequenceAsync(backMiddle);
                                        } else if (tagOfInterest.id == 0) {
                                            TrajectorySequence backSmall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                    .back(1)
                                                    .waitSeconds(1.5)
                                                    .build();
                                            drive.followTrajectorySequenceAsync(backSmall);
                                        } } else {
                                        TrajectorySequence backSmall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                .back(1)
                                                .waitSeconds(1.5)
                                                .build();
                                        drive.followTrajectorySequenceAsync(backSmall);
                                    }

                                })*/
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}