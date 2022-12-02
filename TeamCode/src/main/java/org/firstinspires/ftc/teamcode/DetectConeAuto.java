/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.RedConeDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(preselectTeleOp = "Teleop Field Centric")
public class DetectConeAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    RedConeDetectionPipeline redConeDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!


    // UNITS ARE METERS
    //double tagsize = 0.166;


    AprilTagDetection tagOfInterest = null;
    Pose2d startPose;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Initialize roadrunner

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field
        // This example sets the bot at x: 10, y: 15, and facing 90 degrees (turned counter-clockwise)
        //Pose2d startPose = new Pose2d(10, 15, Math.toRadians(90));
        /*
        // Blue Team
        // Red Corner
        */
        //Pose2d blueTeamRedCornerPose = new Pose2d(36,61.5,Math.toRadians(-90));

        // Blue Corner
        //Pose2d blueTeamBlueCornerPose = new Pose2d(36,61.5,Math.toRadians(-90));
        // Red Team
        // Red Corner
        Pose2d redTeamRedCornerPose = new Pose2d(-36,-61.5,Math.toRadians(90));
        // Blue Corner
        Pose2d redTeamBlueCornerPose = new Pose2d(36,-61.5,Math.toRadians(90));
        // Both
        String selection = "Red Corner";
        startPose = redTeamRedCornerPose;



        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();
        redConeDetectionPipeline = new RedConeDetectionPipeline();

        camera.setPipeline(aprilTagDetectionPipeline);
        //camera.setPipeline(redConeDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        // Init arm
        motorControl.init(hardwareMap);
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == 0 || tag.id == 1 || tag.id == 2) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            if (gamepad1.x) {
                startPose = redTeamRedCornerPose;
                selection = "Red Corner";
            } else if (gamepad1.y) {
                startPose = redTeamBlueCornerPose;
                selection = "Blue Corner";
            }
            telemetry.addLine(selection);
            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        drive.setPoseEstimate(startPose);
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(24)
                .forward(25)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(24)
                .forward(50)
                .strafeRight(24)
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(22)
                .forward(25)
                .build();
        TrajectorySequence tallThenConeRed = drive.trajectorySequenceBuilder(startPose)
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
                    if (tagOfInterest.id == 0) {
                        // TODO: drive to 1 position
                        drive.followTrajectorySequenceAsync(park1);

                    } else if (tagOfInterest.id == 1) {
                        // TODO: drive to 2 position
                        drive.followTrajectorySequenceAsync(park2);
                    } else if (tagOfInterest.id == 2) {
                        // TODO: drive to 3 position
                        drive.followTrajectorySequenceAsync(park3);
                    }
                })
                .build();
        TrajectorySequence tallThenConeBlue = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(12)
                .splineToSplineHeading(new Pose2d(-12, -24, Math.toRadians(0)), Math.toRadians(90))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    // do something half a second before reaching the end of the trajectory
                })
                .addTemporalMarker(() -> {
                    // do something when the trajectory is complete
                })
                .waitSeconds(1)
                .back(6)
                .splineToConstantHeading(new Vector2d(-24, -12), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-60,-12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(1)
                .build();

        /* Actually do something useful */
        if (tagOfInterest == null) {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
            if (selection.equals("Red Corner")) {
                drive.followTrajectorySequenceAsync(tallThenConeRed);
            } else {
                drive.followTrajectorySequenceAsync(tallThenConeBlue);
            }

        } else {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.

            if (tagOfInterest.id == 0) {
                // TODO: drive to 1 position
                drive.followTrajectorySequenceAsync(park1);

            } else if (tagOfInterest.id == 1) {
                // TODO: drive to 2 position
                drive.followTrajectorySequenceAsync(park2);
            } else if (tagOfInterest.id == 2) {
                // TODO: drive to 3 position
                drive.followTrajectorySequenceAsync(park3);
            }
            // do something else
        }
        while (drive.isBusy()) {
            drive.update();
            motorControl.update();
            telemetry.addData("combinedMode", motorControl.currentMode);
            telemetry.addData("armMode", motorControl.currentArmMode);
            telemetry.addData("slidePosition", motorControl.slideTargetPosition);
            telemetry.update();
        }

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


}