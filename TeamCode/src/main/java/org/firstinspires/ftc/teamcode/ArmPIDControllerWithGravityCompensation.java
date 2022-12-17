package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
// AI GENERATED!
@TeleOp(name="Arm PID Controller with Gravity Compensation", group="Linear Opmode")
@Disabled
public class ArmPIDControllerWithGravityCompensation extends LinearOpMode {

    // Declare motors
    private DcMotor armMotor;

    // Declare PID controller variables
    private double kP = 0.01;
    private double kI = 0.001;
    private double kD = 0.1;
    private double error;
    private double integral;
    private double derivative;
    private double lastError;
    private double targetPosition;
    private double output;

    // Declare gravity compensation variables
    private double armMass = 2.0; // kg
    private double armLength = 0.5; // m
    private double gravityConstant = 9.8; // m/s^2
    private double gravityCompensation;

    // Declare timer
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize motors
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        // Wait for start button to be pressed
        waitForStart();

        // Set target position for arm
        targetPosition = 0;

        // Reset timer
        timer.reset();

        // Run PID controller in a loop
        while (opModeIsActive()) {
            // Calculate error
            error = targetPosition - armMotor.getCurrentPosition();

            // Calculate integral
            integral += error * timer.seconds();

            // Calculate derivative
            derivative = (error - lastError) / timer.seconds();

            // Calculate output
            output = kP * error + kI * integral + kD * derivative;

            // Calculate gravity compensation
            gravityCompensation = armMass * gravityConstant * Math.cos(Math.toRadians(armMotor.getCurrentPosition())) / armLength;

            // Set motor power
            armMotor.setPower(output + gravityCompensation);

            // Update last error
            lastError = error;

        }
    }

}
