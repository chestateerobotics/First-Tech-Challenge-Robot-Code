package org.firstinspires.ftc.teamcode;

import java.util.List;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//Program parks in the squares without sensors 

@Autonomous

public class WebcamAutonomous extends LinearOpMode{
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private DcMotor arm;
    private Servo handServo;
    private DistanceSensor rightDistance;

    // Game variables
    private String alliance = "Red";
    private int target;
    private double targetXMin, targetXMax;
    
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private String label;
    
    private static final String VUFORIA_KEY =
            "AXovKxf/////AAABmZ1xSbRDdkXAj9XCHF9C0zpA2yWkekS0x4rJqlFl2RhwdnlZcEvEHJKomiMMZHTvwElLT9SawX7aWdpQZS6L7rfBNI090mCrnoub3lIgwiDTc/DbBuYsz1ICvWHrhz1R0k62/4ZtZXVHclv+ZyqBbbHwI3GfQCgyGSM8aEGdNOEck2Qw8b5WPo8FrtgsqFm4plJ3iIqHvS0Wi3EYosU0eJ46tnvj7xKm3wz0Vf5zUMz4h9fqHZ9WwzXaohSSsad99K42ByO9FAlO1f+vvp2KCr6d5QweT0QGlaSnzz4C03H4X1yARyPIUD9JCeAc8mhhYwqekr7FBdIJLrfp4pKH1nVWdH/J7g0Sa0mJkgDkWwY7";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    
    private ElapsedTime runTime;

    public void runOpMode() {
        runTime = new ElapsedTime();
        initVuforia();
        initTfod();
        
        if (tfod != null) {
            tfod.activate();
        }
        
        telemetry.addData("Ready to start", "");
        telemetry.update();
        Initiate();
        
        waitForStart();
        runTime.reset();
        while (opModeIsActive()) {
            moveForward(0.5);
            sleep(500);
            fullStop();
            while(opModeIsActive() && runTime.seconds() < 5.0){
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                        label = recognition.getLabel();
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                      }
                      telemetry.update();
                    }
                }
            }
            if (label == "Single")
                target = 2;
            else if (label == "Quad")
                target = 3;
            else
                target = 1;
            
            telemetry.addData("Going to", label);
            telemetry.addData("target", target);
            telemetry.update();
            
            if (target == 1) {
                targetXMax = 65;
            } 
            else if (target == 2) {
                targetXMax = 120;
            } 
            else {
                targetXMax = 65;
            }
            
            while(rightDistance.getDistance(DistanceUnit.CM) < 110){
                moveLeft(0.25);
            }
            fullStop();
            sleep(400);
            
            if (target == 1) {
                moveForward(1);
                sleep(1350);
                fullStop();
                sleep(300);
                lineUp(0.2);
                sleep(300);
                moveArm();
                moveLeft(1);
                sleep(400);
                fullStop();
                sleep(300);
                break;
            }
            else if (target == 2){
                moveForward(1);
                sleep(1750);
                fullStop();
                sleep(400);
                lineUp(0.25);
                sleep(500);
                moveArm();
                moveLeft(1);
                sleep(400);
                fullStop();
                sleep(200);
                moveBackward(1);
                sleep(650);
                fullStop();
                sleep(300);
                break;
            }
            else if(target == 3){
                moveForward(1);
                sleep(2100);
                fullStop();
                sleep(500);
                lineUp(0.25);
                sleep(300);
                moveArm();
                sleep(500);
                moveLeft(1);
                sleep(400);
                fullStop();
                sleep(400);
                moveBackward(1);
                sleep(1000);
                fullStop();
                sleep(400);
                break;
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }
    
    private void moveForward(double speed){
        mecanumDrive.driveMecanum(speed, 0, 0);
    }
    
    private void moveBackward(double speed){
        mecanumDrive.driveMecanum(-speed,0,0);
    }
    
    private void moveRight(double speed){
        mecanumDrive.driveMecanum(0,speed,0);
    }
    
    private void moveLeft(double speed){
        mecanumDrive.driveMecanum(0,-speed,0);
    }
    
    private void fullStop(){
        mecanumDrive.driveMecanum(0,0,0);
    }
    
    public void moveArm(){
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(3603);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
        while (arm.isBusy()){
            telemetry.addData("Encoder Arm", arm.getCurrentPosition());
            telemetry.addData("Arm Power", arm.getPower());
            telemetry.update();
        }
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        handServo.setPosition(0);
        sleep(1200);
    }
    
    private void lineUp(double speed){
        while (rightDistance.getDistance(DistanceUnit.CM) > targetXMax) {
            moveRight(speed);
        }
        fullStop();
    }
    
    private void Initiate(){
        mecanumDrive.init(hardwareMap);
        mecanumDrive.setMaxSpeed(0.4);
        mecanumDrive.setEndPowerBehaviorBrake();

        arm = hardwareMap.get(DcMotor.class, "arm_motor");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        handServo = hardwareMap.get(Servo.class, "arm_servo");

        rightDistance = hardwareMap.get(DistanceSensor.class, "RightDistanceSensor");;
    }
    
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
