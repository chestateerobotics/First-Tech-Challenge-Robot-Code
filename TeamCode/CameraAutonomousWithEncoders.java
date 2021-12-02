package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous
public class CameraAutonomousWithEncoders extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "model.tflite";
    private static final String[] LABELS = {
            "cup",
            "water"
    };

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

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private float midPos = 0;

    public void runOpMode() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            //tfod.setZoom(2.5, 16.0/9.0);
        }

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        for (Recognition recognition : updatedRecognitions) {
                            if(recognition.getLabel().equals("cup")){
                                midPos = (recognition.getLeft() + recognition.getRight()) / 2;
                            }
                        }
                    }
                }



                //Go to carousel regardless of what is detected

                if((midPos < 300)){
                    //Go to lower
                    telemetry.addData("Going to lower", "");
                }
                else if (midPos > 700){
                    //Go to top
                    telemetry.addData("Going to top", "");
                }
                else{
                    //Go to middle
                    telemetry.addData("Going to middle", "");
                }
                telemetry.addData("Mid Position", midPos);
                telemetry.update();
                sleep(50000);

                break;
            }
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
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    private void setSpeeds(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        frontLeft.setPower(flSpeed);
        frontRight.setPower(frSpeed);
        backLeft.setPower(blSpeed);
        backRight.setPower(brSpeed);
    }

    private void driveMecanum(double forward, double strafe, double rotate) {
        double frontLeftSpeed = forward + strafe + rotate;
        double frontRightSpeed = forward - strafe - rotate;
        double backLeftSpeed = forward - strafe + rotate;
        double backRightSpeed = forward + strafe - rotate;

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    private double calculateEncoders(double distance){
        int diameter = 96; //In mm
        double encoders = 537.7;
        double revolutions = distance / (diameter * Math.PI);
        return revolutions * encoders;
    }

    private double calculateArcLength(int degree){
        double wheelDiameter = 96; //In mm
        double distanceDiameter = 419.1; //In mm (533.4)
        double encoders = 537.7;
        double arcLength = (degree/360.0) * (distanceDiameter * Math.PI);

        double revs = arcLength / (wheelDiameter * Math.PI);

        return revs * encoders;
    }

    private void moveForward(double distance){
        int num = (int)calculateEncoders(distance);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + num);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + num);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + num);
        backRight.setTargetPosition(backRight.getCurrentPosition() + num);

        frontLeft.setPower(0.3);
        frontRight.setPower(0.3);
        backLeft.setPower(0.3);
        backRight.setPower(0.3);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(300);
    }

    private void moveSide(double distance){
        int num = (int)calculateEncoders(distance);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + num);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - num);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - num);
        backRight.setTargetPosition(backRight.getCurrentPosition() + num);

        frontLeft.setPower(0.3);
        frontRight.setPower(0.3);
        backLeft.setPower(0.3);
        backRight.setPower(0.3);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(300);
    }
    private void doTurn(int degree){
        int num = (int)((degree / 360.0) * 4350);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + num);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - num);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + num);
        backRight.setTargetPosition(backRight.getCurrentPosition() - num);

        frontLeft.setPower(0.3);
        frontRight.setPower(0.3);
        backLeft.setPower(0.3);
        backRight.setPower(0.3);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(300);
    }

    private void move45Right(double distance){
        int num = (int)calculateEncoders(distance * (4.0/3.0));
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + num);
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        backLeft.setTargetPosition(backLeft.getCurrentPosition());
        backRight.setTargetPosition(backRight.getCurrentPosition() + num);

        frontLeft.setPower(0.3);
        frontRight.setPower(0.3);
        backLeft.setPower(0.3);
        backRight.setPower(0.3);

        while(frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(300);
    }

    private void move45Left(double distance){
        int num = (int)calculateEncoders(distance * (4/3.0));
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + num);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + num);
        backRight.setTargetPosition(0);

        frontLeft.setPower(0.2);
        frontRight.setPower(0.2);
        backLeft.setPower(0.2);
        backRight.setPower(0.2);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(300);
    }
}
