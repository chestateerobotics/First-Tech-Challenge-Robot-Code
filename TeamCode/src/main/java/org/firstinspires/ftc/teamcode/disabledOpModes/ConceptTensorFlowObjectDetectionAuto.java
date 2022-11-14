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

package org.firstinspires.ftc.teamcode.disabledOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.classes.MecanumDrive;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Disabled
@Autonomous(name = "AutoCools")
public class ConceptTensorFlowObjectDetectionAuto extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    public DcMotor rightLift;
    public DcMotor leftLift;
    public Servo leftServo;
    public Servo rightServo;
    private double sidePower = 0.5;
    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "traffic3.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "cargill1",
            "eagle3",
            "number2"
    };

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
            "AXovKxf/////AAABmZ1xSbRDdkXAj9XCHF9C0zpA2yWkekS0x4rJqlFl2RhwdnlZcEvEHJKomiMMZHTvwElLT9SawX7aWdpQZS6L7rfBNI090mCrnoub3lIgwiDTc/DbBuYsz1ICvWHrhz1R0k62/4ZtZXVHclv+ZyqBbbHwI3GfQCgyGSM8aEGdNOEck2Qw8b5WPo8FrtgsqFm4plJ3iIqHvS0Wi3EYosU0eJ46tnvj7xKm3wz0Vf5zUMz4h9fqHZ9WwzXaohSSsad99K42ByO9FAlO1f+vvp2KCr6d5QweT0QGlaSnzz4C03H4X1yARyPIUD9JCeAc8mhhYwqekr7FBdIJLrfp4pKH1nVWdH/J7g0Sa0mJkgDkWwY7";
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
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        mecanumDrive.init(hardwareMap);
        initVuforia();
        initTfod();

        /**
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
            //tfod.setZoom(1.0, 16.0/9.0);
        }
        while (!opModeIsActive() && tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    if(recognition.getLabel().equals("cargill1")){
                        sidePower = -0.5;
                    }
                    else if(recognition.getLabel().equals("eagle3")){
                        sidePower = 0.5;
                    }
                    else{
                        sidePower = 0;
                    }

                    telemetry.addData(""," ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                    telemetry.addData("Side: ", sidePower);
                }
                telemetry.update();
            }
            else {
                telemetry.addData("", "no objects");
                telemetry.update();
            }
        }
        /** Wait for the game to begin */
        //telemetry.addData(">", "Press Play to start op mode");
        //telemetry.update();
        waitForStart();
        while(opModeIsActive())
        {
            /*
            mecanumDrive.driveMecanum(0.5,0,0);
            sleep(1320);
            mecanumDrive.driveMecanum(0,0,0);
            sleep(100);
            mecanumDrive.driveMecanum(0,-sidePower,0);
            sleep(105);
            break;
             */

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
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    @Disabled
    @Autonomous
    public static class OdometryAuto extends LinearOpMode
    {
        private MecanumDrive mecanumDrive = new MecanumDrive();
        public DcMotor rightLift;
        public DcMotor leftLift;
        public DcMotor odomSide, odomForward;
        public Servo leftServo;
        public Servo rightServo;
        private double sidePower = 0.5;

        /*
         * Specify the source for the Tensor Flow Model.
         * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
         * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
         * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
         * Here we assume it's an Asset.    Also see method initTfod() below .
         */
        private static final String TFOD_MODEL_ASSET = "traffic3.tflite";
        // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

        private static final String[] LABELS = {
                "cargill1",
                "eagle3",
                "number2"
        };

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
                "AXovKxf/////AAABmZ1xSbRDdkXAj9XCHF9C0zpA2yWkekS0x4rJqlFl2RhwdnlZcEvEHJKomiMMZHTvwElLT9SawX7aWdpQZS6L7rfBNI090mCrnoub3lIgwiDTc/DbBuYsz1ICvWHrhz1R0k62/4ZtZXVHclv+ZyqBbbHwI3GfQCgyGSM8aEGdNOEck2Qw8b5WPo8FrtgsqFm4plJ3iIqHvS0Wi3EYosU0eJ46tnvj7xKm3wz0Vf5zUMz4h9fqHZ9WwzXaohSSsad99K42ByO9FAlO1f+vvp2KCr6d5QweT0QGlaSnzz4C03H4X1yARyPIUD9JCeAc8mhhYwqekr7FBdIJLrfp4pKH1nVWdH/J7g0Sa0mJkgDkWwY7";
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

        public void runOpMode()
        {
            mecanumDrive.init(hardwareMap);
            rightLift = hardwareMap.get(DcMotor.class, "rightLift");
            //rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift = hardwareMap.get(DcMotor.class, "leftLift");
            //leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftServo = hardwareMap.get(Servo.class, "leftServo");
            rightServo = hardwareMap.get(Servo.class, "rightServo");
            rightServo = hardwareMap.get(Servo.class, "rightServo");
            leftServo = hardwareMap.get(Servo.class, "leftServo");

            odomForward = hardwareMap.get(DcMotor.class, "odomForward");
            odomSide = hardwareMap.get(DcMotor.class, "odomSide");

            int distance = 0;

            initTfod();
            initVuforia();

            if (tfod != null) {
                tfod.activate();
                //tfod.setZoom(2.5, 16.0/9.0);
            }
            if (!opModeIsActive()) {
                while (!opModeIsActive()) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Objects Detected", updatedRecognitions.size());

                            // step through the list of recognitions and display image position/size information for each one
                            // Note: "Image number" refers to the randomized image orientation/number
                            for (Recognition recognition : updatedRecognitions) {
                                if(recognition.getLabel().equals("cargill1")){
                                    sidePower = -0.5;
                                }
                                else if(recognition.getLabel().equals("eagle3")){
                                    sidePower = 0.5;
                                }
                                else{
                                    sidePower = 0;
                                }

                                telemetry.addData(""," ");
                                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                                telemetry.addData("Side: ", sidePower);
                            }
                            telemetry.update();
                        }
                    }
                }
            }

            while(opModeIsActive())
            {
                mecanumDrive.driveMecanum(0.5,0,0);
                sleep(1500);
                mecanumDrive.driveMecanum(0,0,0);
                sleep(100);
                mecanumDrive.driveMecanum(0,sidePower,0);
                sleep(1500);
                break;
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
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
            // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        }
    }
}
