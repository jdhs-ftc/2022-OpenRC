/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "TFOD Everyday Objects", group = "Concept")
//@Disabled
public class ConceptTensorFlowObjectDetectionWebcam extends LinearOpMode {
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/detect.tflite";
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
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // read the label map text files.
        readLabels();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //Camera Webcam =
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(tfod, 0);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        int detectedLayer = 0; // 1 is lowest, 2 is middle, 3 is highest
        if (opModeIsActive() && tfod != null) {
            while (opModeIsActive()) { // TODO: add "& detectedLayer == 0" after we have the location of the detections set
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {


                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        float leftEdge = recognition.getLeft();
                        if (leftEdge < 10 && leftEdge > 1) { // TODO: edit to be accurate
                            detectedLayer = 1;
                        } else if (leftEdge > 10 && leftEdge < 20) {
                            detectedLayer = 2;
                        } else if (leftEdge > 20 && leftEdge < 30) {
                            detectedLayer = 3;

                        }
                        telemetry.addData("Detected Layer", detectedLayer);
                        // step through the list of recognitions and display boundary info
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                leftEdge, recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        telemetry.update();

                    }
                } else {
                    telemetry.addData("Error", "TFOD has initialized but no objects have been detected");
                    telemetry.update();
                }
            }


        }
        // TODO: roadrunner paths



        if (tfod != null) {
            tfod.shutdown();
        }
    }


    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        if(labels != null) {
            tfod.loadModelFromFile(TFOD_MODEL_FILE, labels);
        }
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