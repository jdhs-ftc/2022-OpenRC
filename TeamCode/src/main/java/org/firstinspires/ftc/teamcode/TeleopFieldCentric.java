package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BlueConeDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.PoleDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.RedConeDetectionPipeline;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(name = "Teleop Field Centric")
@Config
public class TeleopFieldCentric extends LinearOpMode {
    private static final PIDFController armController = new PIDFController(new PIDCoefficients(0.1, 0, 0));
    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private final PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    DcMotorEx slide;
    DcMotorEx arm;
    ColorSensor color;
    DigitalChannel magnet;
    double slideTargetPosition;
    double slideError;
    CRServo claw;
    double speed;
    double armTargetPosition;
    double armError;
    double slidePeakCurrentAmps;
    String hubNames;
    OpenCvCamera camera;
    RedConeDetectionPipeline redConeDetectionPipeline;
    BlueConeDetectionPipeline blueConeDetectionPipeline;
    PoleDetectionPipeline poleDetectionPipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            hubNames = hubNames + hardwareMap.getNamesOf(hub);
        }


        //Initialization Period
        Mode armMode = Mode.DOWN;
        // RoadRunner Init
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);
        //blue = PoseStorage.blue;

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);
        armController.setOutputBounds(-0.75, 0.75);

        /* Motor Init */

        // Initiate Slide
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setCurrentAlert(8, CurrentUnit.AMPS);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initiate Arm
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setCurrentAlert(4, CurrentUnit.AMPS);
        magnet = hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);
        color = hardwareMap.get(ColorSensor.class, "color");


        // Initiate Claw
        claw = hardwareMap.get(CRServo.class, "claw");
        claw.setPower(0);

        // Vision
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        blueConeDetectionPipeline = new BlueConeDetectionPipeline();
        redConeDetectionPipeline = new RedConeDetectionPipeline();
        poleDetectionPipeline = new PoleDetectionPipeline();

        camera.setPipeline(poleDetectionPipeline);
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
        FtcDashboard.getInstance().startCameraStream(camera, 15);
        /* Variable Init */
        slideTargetPosition = 0.0;
        armTargetPosition = 0.0;
        speed = .8;
        slidePeakCurrentAmps = 0.0;

        waitForStart();

        if (isStopRequested()) return;


        //Run Period


        while (opModeIsActive() && !isStopRequested()) {
            // Update the controller input bounds
            if (gamepad1.left_bumper) {
                speed = .25;
            } else if (gamepad1.right_bumper) {
                speed = 1;
            } else {
                speed = .5;
            }


            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed
            );
            Pose2d poseEstimate = drive.getPoseEstimate();
            input = input.rotated(-poseEstimate.getHeading() + Math.toRadians(90.0));


            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            Vector2d controllerHeading = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
            if (controllerHeading.distTo(new Vector2d(0.0, 0.0)) < 0.7) {

                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                (gamepad1.left_trigger - gamepad1.right_trigger)
                        )
                );
            } else {
                // Set the target heading for the heading controller to our desired angle


                headingController.setTargetPosition(controllerHeading.angle() + Math.toRadians(90.0));


                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV)
                        * DriveConstants.TRACK_WIDTH;
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                headingInput
                        ));

            }
            if (gamepad1.right_stick_button) {
                headingController.setTargetPosition(drive.getExternalHeading() + poleDetectionPipeline.getMaxRect().x);

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV)
                        * DriveConstants.TRACK_WIDTH;
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                headingInput
                        ));
            }
            if (armMode != Mode.MOVING_DOWN && color.red() < 2500 && color.blue() < 2500) {
                claw.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
            } else {
                claw.setPower(0.25 + gamepad2.left_trigger - gamepad2.right_trigger);
            }
            //arm.setPower(gamepad2.right_stick_x * 0.5);


            // Update everything. Odometry. Etc.
            drive.update();

            //FieldcENTRIC
            if (gamepad1.dpad_down && gamepad1.dpad_left && gamepad1.dpad_up && gamepad1.dpad_right) {
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90.0)));
            }

            // Slide
            slideTargetPosition = slideTargetPosition + (-gamepad2.left_stick_y * 10);
            if (gamepad2.y) {
                slideTargetPosition = 1200;
                armMode = Mode.MOVING_UP;
            }
            if (gamepad2.b) {
                slideTargetPosition = 400;
                armMode = Mode.MOVING_UP;
            }
            if (gamepad2.a) {
                slideTargetPosition = 20;
                armMode = Mode.MOVING_DOWN;
            }
            if (gamepad2.x) {
                slideTargetPosition = 1200;
                armMode = Mode.MOVING_DOWN;
            }
            if (gamepad2.dpad_right && gamepad2.dpad_left) {
                armMode = Mode.DOWN;
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (slideTargetPosition > 1100) {
                slideTargetPosition = 1100;
            } else if (slideTargetPosition < 0) {
                slideTargetPosition = 0;
            }


            // overly complex slide code

            // obtain the encoder position and calculate the error
            slideError = slideTargetPosition - slide.getCurrentPosition();
            slide.setTargetPosition((int) slideTargetPosition);
            slide.setTargetPositionTolerance(10);
            if (slideError > 0) {
                slide.setPower(0.8);
            } else {
                slide.setPower(-0.8);
            }
            if (!slide.isOverCurrent() && !(gamepad2.left_stick_y > 0)) {
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide.setPower(gamepad1.left_stick_y);
                slideTargetPosition = slide.getCurrentPosition();
            }
            // update peak current if larger then previous
            if (slide.getCurrent(CurrentUnit.AMPS) > slidePeakCurrentAmps) {
                slidePeakCurrentAmps = slide.getCurrent(CurrentUnit.AMPS);
            }


            // arm
            /*
            armController.setTargetPosition(armTargetPosition);
            // make sure arm is not overcurrent
            if (!arm.isOverCurrent()) {
                arm.setPower(0);
                //arm.setPower(armController.update(arm.getCurrentPosition()));
            } else {
                arm.setPower(0);
            }
             */




            switch (armMode) {
                case UP:
                    arm.setPower(-gamepad2.right_stick_y * 0.5);
                    break;
                case DOWN:
                    arm.setPower(-gamepad2.right_stick_y * 0.5);
                    break;
                case MOVING_UP:
                    if (arm.getCurrentPosition() >= 350 || gamepad2.x) {
                        armMode = Mode.UP;
                        arm.setPower(0);
                    } else {
                        if (slide.getCurrentPosition() > 100 && !arm.isOverCurrent()) {

                            arm.setPower(0.75);
                            // compensate for arm gravity using the arm angle, weight, and length
                            //arm.setPower((0.75 * Math.cos(Math.toRadians(arm.getCurrentPosition() / 2.0))) + 0.75);
                        } else {
                            arm.setPower(0);
                        }
                    }
                    break;
                case MOVING_DOWN:
                    if (arm.getCurrentPosition() <= 5) {
                        armMode = Mode.DOWN;
                        arm.setPower(0);
                    } else {
                        arm.setPower(-0.25);
                    }
                    if ((gamepad2.left_trigger == 0) && (gamepad2.right_trigger == 0)) {
                        claw.setPower(0.1);
                    } else {
                        claw.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                    }

                    break;
            }
            // if right bumper on gamepad2 pressed, reset the slide encoder

            // color




            // Vision
            Rect maxRect = redConeDetectionPipeline.getMaxRect();
            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("armPosition", arm.getCurrentPosition());
            telemetry.addData("armTargetPosition", armTargetPosition);
            telemetry.addData("armCurrent", arm.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("slidePeakCurrent", slidePeakCurrentAmps);
            telemetry.addData("slideTargetPosition", slideTargetPosition);
            telemetry.addData("slidePosition", slide.getCurrentPosition());
            telemetry.addData("controllerHeading", controllerHeading.angle());
            telemetry.addData("Magnet", magnet.getState());
            telemetry.addData("Armstate", armMode);
            telemetry.addData("hubNames", hubNames);
            telemetry.addData("maxRect", maxRect);
            telemetry.addData("colorBlue", color.blue());
            telemetry.addData("colorRed", color.red());
            telemetry.addData("servoPower", claw.getPower());
            telemetry.update();
        }
    }

    enum Mode {
        UP,
        MOVING_UP,
        MOVING_DOWN,
        DOWN
    }
}