package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(group = "advanced")
@Disabled
public class StateMachineWithObjectRecognition extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        LOOK_FOR_PIECE,
        DRIVE_CLOCK,
        DRIVE_STOP,
        DRIVE_TRAFFIC_LIGHT,
        /* TODO: add our own states here
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TURN_1,         // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        TURN_2,         // Finally, we're gonna turn again

         */
        IDLE            // Our bot will enter the IDLE state when done
    }
    enum detectedObject {
        NONE,
        CLOCK,
        STOP,
        TRAFFIC_LIGHT
    }
    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;
    detectedObject currentObject = detectedObject.NONE;
    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(15, 10, Math.toRadians(180));
    @SuppressLint("SdCardPath")
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/detect.tflite";
    @SuppressLint("SdCardPath")
    private static final String TFOD_MODEL_LABELS = "/sdcard/FIRST/tflitemodels/labelmap.txt";
    private String[] labels;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AYIy+wf/////AAABmTogX7sfc00thsy7eGmWjM0t4M0Us8RBEMt1Iirw/kewa0thLqGGvBQ6ywDiCn6A6FxGh8OveZuemqV17zZezDrUWcQ2CNl2hUo0HUm5Lq4X9UPvlqLd7CTp7yWrRkJS7Wz3V2Balxyuq06cRnWDv/IegCK88mlrtMiC677QXo4k5SfBlhKJtmUCF2xCxeudF6tUvsigoYnfW5J924saoNiQJKagpfAxoTey8o2/AaC8Gy3UYaQjs3ye29LpELDyyxTGAWYRgsKWXcpP7jQtbsQMqslY5UUqUIBcI0BcnYZ3iZkgDPf7pfXhs1zyxAnoE+GKPPDg/eOAn7G6Rd+JTasXb+tkhT7v73DcAzJxh0y1";

    /**
     * vuforia is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * tfod is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        Lift lift = new Lift(hardwareMap);

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);
        /* TODO: define trajectories
        // Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(45, -20), Math.toRadians(90))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())enum
                .lineTo(new Vector2d(45, 0))
                .build();

        // Define the angle to turn at
        double turnAngle1 = Math.toRadians(-270);

        // Third trajectory
        // We have to define a new end pose because we can't just call trajectory2.end()
        // Since there was a point turn before that
        // So we just take the pose from trajectory2.end(), add the previous turn angle to it
        Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));
        Trajectory trajectory3 = drive.trajectoryBuilder(newLastPose)
                .lineToConstantHeading(new Vector2d(-15, 0))
                .build();

        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);
        */

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(tfod, 0);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.LOOK_FOR_PIECE;
        //drive.followTrajectoryAsync(trajectory1); TODO: replace this

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case LOOK_FOR_PIECE:
                    // If we see a piece, then we want to decide which direction to go to
                    // We can use the isDetected() method to check if we see a piece
                    if (tfod != null && tfod.getRecognitions().size() > 0) {
                        // We see a piece, so let's decide which face is towards us on it
                        String label = tfod.getRecognitions().get(0).getLabel();
                        // And use a switch statement to decide which state to set
                        /* TODO: do that
                        switch (label) {
                            case "Quad":
                                // We see a quad, so we want to go to the quad state
                                currentState = State.GO_TO_QUAD;
                                break;
                            case "Single":
                                // We see a single, so we want to go to the single state
                                currentState = State.GO_TO_SINGLE;
                                break;
                            case "None":
                                // We see a none, so we want to go to the none state
                                currentState = State.GO_TO_NONE;
                                break;
                        }
                         */

                    }
                /* Example trajectories removed TODO: add our own
                case TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TURN_1;
                        drive.turnAsync(turnAngle1);
                    }
                    break;
                case TURN_1:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectoryAsync(trajectory3);
                    }
                    break;
                case TRAJECTORY_3:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.TURN_2;
                        drive.turnAsync(turnAngle2);
                    }
                    break;
                case TURN_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;

                 */

                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }
            List<Recognition> updatedRecognitions = null;
            if (tfod != null) {
                updatedRecognitions = tfod.getUpdatedRecognitions();
            }
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    String label = recognition.getLabel();
                    //
                    telemetry.addData("Found:", label);

                }
            } else {
                telemetry.addData("Error", "TFOD has initialized but no objects have been detected");
                telemetry.update();
            }
            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            //PoseStorage.currentPose = poseEstimate; TODO: fix this

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    static class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware
        }

        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift
        }
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, labels);
    }
    /**
     * Read the labels for the object detection model from a file.
     */
    private void readLabels() {
        ArrayList<String> labelList = new ArrayList<>();

        // try to read in the the labels.
        try (BufferedReader br = new BufferedReader(new FileReader(TFOD_MODEL_LABELS))) {
            int index = 0;
            while (br.ready()) {
                /* skip the first row of the labelmap.txt file.
                 * if you look at the TFOD Android example project (https://github.com/tensorflow/examples/tree/master/lite/examples/object_detection/android)
                 * you will see that the labels for the inference model are actually extracted (as metadata) from the .tflite model file
                 * instead of from the labelmap.txt file. if you build and run that example project, you'll see that
                 * the label list begins with the label "person" and does not include the first line of the labelmap.txt file ("???").
                 * i suspect that the first line of the labelmap.txt file might be reserved for some future metadata schema
                 * (or that the generated label map file is incorrect).
                 * for now, skip the first line of the label map text file so that your label list is in sync with the embedded label list in the .tflite model.
                 */
                if(index == 0) {
                    // skip first line.
                    br.readLine();
                } else {
                    labelList.add(br.readLine());
                }
                index++;
            }
        } catch (Exception e) {
            telemetry.addData("Exception", e.getLocalizedMessage());
            telemetry.update();
        }

        if (labelList.size() > 0) {
            labels = getStringArray(labelList);
            RobotLog.vv("readLabels()", "%d labels read.", labels.length);
            for (String label : labels) {
                RobotLog.vv("readLabels()", " " + label);
            }
        } else {
            RobotLog.vv("readLabels()", "No labels read!");
        }
    }

    // Function to convert ArrayList<String> to String[]
    private String[] getStringArray(ArrayList<String> arr)
    {
        // declaration and initialize String Array
        String[] str = new String[arr.size()];

        // Convert ArrayList to object array
        Object[] objArr = arr.toArray();

        // Iterating and converting to String
        int i = 0;
        for (Object obj : objArr) {
            str[i++] = (String)obj;
        }

        return str;
    }
}