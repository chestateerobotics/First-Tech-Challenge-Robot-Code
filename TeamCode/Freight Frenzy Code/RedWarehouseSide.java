package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@Autonomous
public class RedWarehouseSide extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor arm_right;
    private DcMotor arm_left;
    private DcMotor intake;
    private DcMotor carousel;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean goWarehouse = false;
    private double maxSpeed = 0.4;
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
    
    private float midPos = 1000;
    private int level = 1;
    private int allianceHubDistance;
    private boolean foundCup = false;

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        
        arm_right = hardwareMap.get(DcMotor.class, "arm_right");
        arm_left = hardwareMap.get(DcMotor.class, "arm_left");
        carousel = hardwareMap.get(DcMotor.class, "carousel_arm");
        intake = hardwareMap.get(DcMotor.class, "intake_arm");

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

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        arm_right.setDirection(DcMotor.Direction.REVERSE);
        
        arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
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

        telemetry.addData("Going to Warehouse", goWarehouse);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){
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
                            foundCup = true;
                        }
                    }
                }
            }



            //Go to carousel regardless of what is detected
            if((midPos < 500) && foundCup){
                //Go to lower
                telemetry.addData("Going to Lower", "");
                level = 1;
                allianceHubDistance = 650;
            }
            else if ((midPos >= 500) && foundCup){
                //Go to top
                telemetry.addData("Going to Middle", "");
                level = 2;
                allianceHubDistance = 700;
            }
            else{
                allianceHubDistance = 750;
                level = 3;
            }
            telemetry.addData("Mid Position", midPos);
            telemetry.update();
            moveArm(0.4);
            sleep(400);
            moveArm(0.1);
            encoderArm(level, 1);
            encoderArm(level,1);
            moveSide(-526);
            moveForward(allianceHubDistance);
            intake.setPower(-0.3);
            sleep(2000);
            intake.setPower(0);
            moveForward(-allianceHubDistance);
            
            moveSide(1300);
            moveForward(700);
            telemetry.addData("Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", frontRight.getCurrentPosition());
            telemetry.addData("Back Left", backLeft.getCurrentPosition());
            telemetry.addData("Back Right", backRight.getCurrentPosition());
            telemetry.update();
            break;
        }
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

        frontLeft.setPower(maxSpeed);
        frontRight.setPower(maxSpeed);
        backLeft.setPower(maxSpeed);
        backRight.setPower(maxSpeed);

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

        frontLeft.setPower(maxSpeed);
        frontRight.setPower(maxSpeed);
        backLeft.setPower(maxSpeed);
        backRight.setPower(maxSpeed);

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

        frontLeft.setPower(maxSpeed);
        frontRight.setPower(maxSpeed);
        backLeft.setPower(maxSpeed);
        backRight.setPower(maxSpeed);

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

        frontLeft.setPower(maxSpeed);
        frontRight.setPower(maxSpeed);
        backLeft.setPower(maxSpeed);
        backRight.setPower(maxSpeed);

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
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + num);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + num);
        backRight.setTargetPosition(backRight.getCurrentPosition());

        frontLeft.setPower(maxSpeed);
        frontRight.setPower(maxSpeed);
        backLeft.setPower(maxSpeed);
        backRight.setPower(maxSpeed);

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
    
    private void moveArm(double speed){
        arm_left.setPower(speed);
        arm_right.setPower(speed);
    }
    
    private void encoderArm(int level, int time){
        if(level == 1){
            arm_left.setTargetPosition(350);
            arm_right.setTargetPosition(350);
        }
        else if(level == 2){
            arm_left.setTargetPosition(745);
            arm_right.setTargetPosition(745);
        }
        else{
            arm_left.setTargetPosition(1060);
            arm_right.setTargetPosition(1060);
        }

        arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_left.setPower(0.4);
        arm_right.setPower(0.4);
        runtime.reset();
        while ((arm_right.isBusy() || arm_left.isBusy()) || (runtime.seconds() <= time)){
            telemetry.addData("Runtime Seconds = ", runtime.seconds());
            telemetry.update();
        }
        arm_left.setPower(0.1);
        arm_right.setPower(0.1);
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
}