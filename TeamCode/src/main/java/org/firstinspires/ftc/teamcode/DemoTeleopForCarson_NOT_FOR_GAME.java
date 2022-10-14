package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class DemoTeleopForCarson_NOT_FOR_GAME extends LinearOpMode {
    DcMotorEx slide;
    double slideTargetPosition;
    double error;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide = hardwareMap.get(DcMotorEx.class, "arm");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideTargetPosition = 0.0;

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.5,
                            -gamepad1.left_stick_x * 0.5,
                            -gamepad1.right_stick_x * 0.5
                    )
            );

            drive.update();

            slideTargetPosition = slideTargetPosition + (-gamepad1.right_stick_y * 10);
            if (gamepad1.y) {
                slideTargetPosition = 1200;
                // move arm
            }
            if (gamepad1.b) {
                slideTargetPosition = 600;
            }
            if (gamepad1.a) {
                slideTargetPosition = 10;
            }
            if (slideTargetPosition > 1200) {
                slideTargetPosition = 1200;
            } else if (slideTargetPosition < 0) {
                slideTargetPosition = 0;
            }

            error = slideTargetPosition - slide.getCurrentPosition();
            // from https://www.ctrlaltftc.com/introduction-to-closed-loop-control TODO: add to notebook
            if (Math.abs(error) > 10) {
                // obtain the encoder position

                // calculate the error

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



            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("armPosition", slide.getCurrentPosition());
            telemetry.addData("armTargetPosition", slideTargetPosition);
            telemetry.addData("error", error);
            telemetry.update();
        }
    }
}
