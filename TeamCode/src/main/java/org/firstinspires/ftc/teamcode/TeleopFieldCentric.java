package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp()
public class TeleopFieldCentric extends LinearOpMode {
    DcMotorEx slide;
    double slideTargetPosition;
    double error;
    CRServo claw;
    boolean fieldCentricEnable;
    boolean blue;
    double speed;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialization Period

        // RoadRunner Init
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        // Motor Init
        // Arm
        slide = hardwareMap.get(DcMotorEx.class, "arm");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Claw
        claw = hardwareMap.get(CRServo.class, "claw");
        claw.setPower(0);

        // Variable Init
        fieldCentricEnable = true;
        slideTargetPosition = 0.0;
        blue = true;
        speed = .8;

        waitForStart();

        if (isStopRequested()) return;


        //Run Period


        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            if (gamepad1.left_bumper){
                speed = .25;
            } else if (gamepad2.right_bumper){
                speed = 1;
            } else {
                speed = .5;
            }
            if (gamepad1.x) {
                blue = !blue;
            }


            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed
            );
            if (fieldCentricEnable) {
                if (blue) {
                    input = input.rotated(-poseEstimate.getHeading() + Math.toRadians(90.0));
                } else {
                    input = input.rotated(-poseEstimate.getHeading() + Math.toRadians(270.0));
                }
            }

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            (gamepad1.left_trigger - gamepad1.right_trigger)
                    )
            );

            claw.setPower(gamepad2.left_trigger - gamepad2.right_trigger);


            // Update everything. Odometry. Etc.
            drive.update();

            //FieldcENTRIC

            // Slide
            slideTargetPosition = slideTargetPosition + (-gamepad2.left_stick_y * 10);
            if (gamepad2.y) {
                slideTargetPosition = 1200;
                // TODO: move arm
            }
            if (gamepad2.b) {
                slideTargetPosition = 600;
            }
            if (gamepad2.a) {
                slideTargetPosition = 20;
            }
            if (slideTargetPosition > 1200) {
                slideTargetPosition = 1200;
            } else if (slideTargetPosition < 0) {
                slideTargetPosition = 0;
            }


            // obtain the encoder position and calculate the error
            error = slideTargetPosition - slide.getCurrentPosition();
            // from https://www.ctrlaltftc.com/introduction-to-closed-loop-control TODO: add to notebook
            if (Math.abs(error) > 10) {


                // set motor power proportional to the error
                if (Math.abs(error) > 50) {
                    if (error > 0) {
                        slide.setPower(0.5);
                    } else {
                        slide.setPower(-0.5);
                    }
                } else {
                    slide.setPower(error / 100);
                }
            }


            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("armPosition", slide.getCurrentPosition());
            telemetry.addData("armTargetPosition", slideTargetPosition);
            telemetry.addData("toggleFieldCentric", fieldCentricEnable);
            telemetry.addData("blue", blue);
            telemetry.update();
        }
    }
}