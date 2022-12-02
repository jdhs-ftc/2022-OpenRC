package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class motorControl {
    public static DcMotorEx arm;
    public static DcMotorEx slide;
    enum armMode {
        UP,
        MOVING_UP,
        MOVING_DOWN,
        DOWN
    }
    enum combinedMode {
        TOP,
        MIDDLE,
        BOTTOM
    }
    static double slideTargetPosition;
    static armMode currentArmMode;
    static combinedMode currentMode;
    public static void init(@NonNull HardwareMap hardwareMap) {
        currentArmMode = armMode.DOWN;
        currentMode = combinedMode.BOTTOM;
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setCurrentAlert(4, CurrentUnit.AMPS);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setCurrentAlert(8, CurrentUnit.AMPS);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void setArmPower(double power){
        arm.setPower(power);
    }
    public static void armForceStop() {
        if (currentArmMode == armMode.MOVING_UP) {
            currentArmMode = armMode.UP;
        } else if (currentArmMode == armMode.MOVING_DOWN) {
            currentArmMode = armMode.DOWN;
        }

    }
    public static void setArmMode(armMode newMode) {
        currentArmMode = newMode;
    }
    public static void armUpdate() {
        switch (currentArmMode) {
            case UP:
                arm.setPower(0);
                break;
            case DOWN:
                arm.setPower(0);
                break;
            case MOVING_UP:
                if (arm.getCurrentPosition() >= 360) {
                    currentArmMode = armMode.UP;
                    arm.setPower(0);
                } else {
                    arm.setPower(0.75);
                }
                break;
            case MOVING_DOWN:
                if (arm.getCurrentPosition() <= 5) {
                    currentArmMode = armMode.DOWN;
                    arm.setPower(0);
                } else {
                    arm.setPower(-0.25);
                }

                break;
        }
    }

    public static void setSlideTargetPosition(double newTarget) {
        slideTargetPosition = newTarget;
    }
    public static void slideUpdate() {
        // overly complex slide code
        // obtain the encoder position and calculate the error
        double slideError = slideTargetPosition - slide.getCurrentPosition();
        slide.setTargetPosition((int) slideTargetPosition);
        slide.setTargetPositionTolerance(10);
        if (slideError > 0) {
            slide.setPower(0.8);
        } else {
            slide.setPower(-0.8);
        }
        if (!slide.isOverCurrent()) {
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideTargetPosition = slide.getCurrentPosition();
        }
    }

    public static void setMode(combinedMode newMode) {
        currentMode = newMode;
    }
    public static void update() {
        switch (currentMode) {
            case BOTTOM:
                currentArmMode = armMode.MOVING_DOWN;
                slideTargetPosition = 0;
                break;
            case MIDDLE:
                currentArmMode = armMode.MOVING_UP;
                slideTargetPosition = 0;
                break;
            case TOP:
                currentArmMode = armMode.MOVING_UP;
                slideTargetPosition = 1190;
                break;

        }
        slideUpdate();
        armUpdate();
    }
}
