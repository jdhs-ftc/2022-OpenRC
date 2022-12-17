package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class motorControl {

    public static combinedMode getCurrentMode() {
        return currentMode;
    }

    public static void setCurrentMode(combinedMode currentMode) {
        motorControl.currentMode = currentMode;
    }

    enum combinedMode {
        TOP,
        MIDDLE,
        BOTTOM
    }

    private static combinedMode currentMode;
    public static void init(@NonNull HardwareMap hardwareMap) {
        arm.init(hardwareMap);
        slide.init(hardwareMap);
        claw.init(hardwareMap);

        setCurrentMode(combinedMode.BOTTOM);
    }

    public static void setMode(combinedMode newMode) {
        setCurrentMode(newMode);
    }
    public static void update() {
        switch (getCurrentMode()) {
            case BOTTOM:
                if (arm.mode != arm.armMode.DOWN) {
                    arm.mode = arm.armMode.MOVING_DOWN;
                    slide.targetPosition = 0;
                }

                break;
            case MIDDLE:
                if (arm.mode != arm.armMode.UP) {
                    arm.mode = arm.armMode.MOVING_UP;
                }
                slide.targetPosition = 0;
                break;
            case TOP:
                if (arm.mode != arm.armMode.UP) {
                    arm.mode = arm.armMode.MOVING_UP;
                    slide.targetPosition = 1100;
                }

                break;

        }
        slide.update();
        arm.armUpdate();
    }

    public static class arm {
        public static DcMotorEx motor;
        static armMode mode;

        public static void init(@NonNull HardwareMap hardwareMap) {
            mode = armMode.DOWN;
            motor = hardwareMap.get(DcMotorEx.class, "arm");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(4, CurrentUnit.AMPS);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public static armMode getMode() {
            return mode;
        }
        
        public static void setPower(double power) {
            motor.setPower(power);
        }

        public static void armForceStop() {
            if (mode == armMode.MOVING_UP) {
                mode = armMode.UP;
            } else if (mode == armMode.MOVING_DOWN) {
                mode = armMode.DOWN;
            }

        }

        public static void setMode(armMode newMode) {
            mode = newMode;
        }

        public static void armUpdate() {
            switch (mode) {
                case UP:
                    motor.setPower(0);
                    break;
                case DOWN:
                    motor.setPower(0);
                    break;
                case MOVING_UP:
                    if (motor.getCurrentPosition() >= 350) {
                        mode = armMode.UP;
                        motor.setPower(0);
                    } else {
                        motor.setPower(0.75);
                    }
                    break;
                case MOVING_DOWN:
                    if (motor.getCurrentPosition() <= 5) {
                        mode = armMode.DOWN;
                        motor.setPower(0);
                    } else {
                        motor.setPower(-0.25);
                    }

                    break;
            }
        }

        enum armMode {
            UP,
            MOVING_UP,
            MOVING_DOWN,
            DOWN
        }
    }

    public static class slide {
        public static DcMotorEx motor;
        static double targetPosition;
        
        public static void init(@NonNull HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "slide");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(8, CurrentUnit.AMPS);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
        }


        public static double getTargetPosition() {
            return targetPosition;
        }
        
        public static void setTargetPosition(double newTarget) {
            targetPosition = newTarget;
        }

        public static void update() {
            // overly complex slide code
            // obtain the encoder position and calculate the error
            double slideError = targetPosition - motor.getCurrentPosition();
            motor.setTargetPosition((int) targetPosition);
            motor.setTargetPositionTolerance(10);
            if (slideError > 0) {
                motor.setPower(0.8);
            } else {
                motor.setPower(-0.5);
            }
            if (!motor.isOverCurrent()) {
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetPosition = motor.getCurrentPosition();
            }
        }
    }

    public static class claw {
        public static CRServo servo;

        public static void init(HardwareMap hardwareMap) {
            servo = hardwareMap.get(CRServo.class, "claw");
            servo.setPower(0);
        }

        public static void setPower(double power) {
            servo.setPower(power);
        }
    }
}
