package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
import java.util.Queue;
@Autonomous
public class BlueWarehouseSideAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "GreenCup.tflite";
    private static final String[] LABELS = {
            "cup"
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

    private float midPos =  1000;
    private int level = 3;
    private double allianceHubDistance;
    private boolean foundCup = false;

    private DcMotor arm_right;
    private DcMotor arm_left;
    private DcMotor intake;
    private DcMotor carousel;

    private ColorSensor colorsensor;
    private DigitalChannel red;
    private DigitalChannel green;

    private double armSpeed = .3;
    private int i = 0;


    private ElapsedTime armRuntime = new ElapsedTime();
    private ElapsedTime robotRuntime = new ElapsedTime();

    Queue<Trajectory> trajectoryQueue;
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d currentPose = new Pose2d();
        drive.setPoseEstimate(startPose);

        arm_right = hardwareMap.get(DcMotor.class, "arm_right");
        arm_left = hardwareMap.get(DcMotor.class, "arm_left");
        carousel = hardwareMap.get(DcMotor.class, "carousel_arm");
        intake = hardwareMap.get(DcMotor.class, "intake_arm");

        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");
        red = hardwareMap.get(DigitalChannel.class, "red");
        green = hardwareMap.get(DigitalChannel.class, "green");

        arm_right.setDirection(DcMotor.Direction.REVERSE);

        arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);

        //I am sorry for the mess...
        //Line up with the alliance hub and close to the y position
        //Moves arm while moving
        Trajectory hubLineUp = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(20.5, 15),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, () -> {
                    // Turn on motor
                    if(i == 0) {
                        encoderArm(level, 0.5);
                    }
                    else{
                        encoderArm(3, 0.5);
                    }
                })
                .addTemporalMarker(3, () -> {
                    // Run action at 1.5 seconds  after the previous marker

                    // Turn off motor
                    encoderArm(level, 0.0);
                })
                .build();
/*
        //Finish Line Up
        Trajectory finishLineUp = drive.trajectoryBuilder(new Pose2d())
                .forward(allianceHubDistance)
                .build();
*/
        //Go to start Position so we can go to warehouse
        //Make sure arm is high enough to not hit the wall
        Trajectory origin = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(5, 0, Math.toRadians(90)))
                .addTemporalMarker(0, () -> {
                    // Turn on motor
                    encoderArm(2, 0.5);
                })
                .addTemporalMarker(1.5, () -> {
                    // Run action at 1.5 seconds  after the previous marker

                    // Turn off motor
                    encoderArm(2, 0.0);
                })

                .build();

        //Go into warehouse
        Trajectory insideWarehouse = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-32.625, 0))
                .build();

/*
        //Grab block parallel
        Trajectory parallel = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 17, Math.toRadians(0)))
                .build();
*/







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
        while(!opModeIsActive() && tfod != null){
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

            if((midPos < 500 && midPos >= 100) && foundCup){
                //Go to middle
                telemetry.addData("Going to Middle", "");
                level = 2;
                allianceHubDistance = 5.5;
            }
            else if ((midPos >= 500 && midPos <= 900) && foundCup){
                //Go to top
                telemetry.addData("Going to Top", "");
                allianceHubDistance = 10.5;
                level = 3;
            }
            else{
                //Go to lower
                telemetry.addData("Going to Lower", "");
                level = 1;
                allianceHubDistance = 4;
            }

            telemetry.addData("Mid Position", midPos);
            telemetry.update();
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        armRuntime.reset();
        robotRuntime.reset();
        while(opModeIsActive()) {
            /*
                Measurements to note:
                    One floor tile: 21.75 in
             */
            encoderArm(1,0.5);
            sleep(300);
            encoderArm(1,0.0);
            drive.followTrajectory(hubLineUp);

            currentPose = drive.getPoseEstimate();

            //Finish Line Up
            Trajectory finishLineUp = drive.trajectoryBuilder(currentPose)
                    .forward(allianceHubDistance)
                    .build();
            sleep(100);
            drive.followTrajectory(finishLineUp);
            dropBlock();
            drive.followTrajectory(origin);
            drive.followTrajectory(insideWarehouse);
            //Grab block perpendicular
            currentPose = drive.getPoseEstimate();
            Trajectory perpendicular = drive.trajectoryBuilder(currentPose)
                    .lineToLinearHeading(new Pose2d(-31.5, 19.5, Math.toRadians(225)))
                    .addTemporalMarker(0, () -> {
                        // Turn on motor
                        encoderArm(0, 0.2);
                    })
                    .addTemporalMarker(1.5, () -> {
                        // Run action at 1.5 seconds  after the previous marker

                        // Turn off motor
                        encoderArm(0, 0.0);
                    })
                    .build();
            drive.followTrajectory(perpendicular);

            drive.setMotorPowers(0.5,0.5,0.5,0.5);
            armRuntime.reset();
            while(colorsensor.red() < 200 && armRuntime.seconds() < 5){
                intake.setPower(0.8);
            }
            intake.setPower(0);
            drive.setMotorPowers(0,0,0,0);

            currentPose = drive.getPoseEstimate();
            Trajectory moveBack = drive.trajectoryBuilder(currentPose)
                    .back(2)
                    .build();

            drive.followTrajectory(moveBack);
            intake.setPower(0.4);
            sleep(400);
            intake.setPower(0);
            currentPose = drive.getPoseEstimate();
            Trajectory warehouse = drive.trajectoryBuilder(currentPose)
                    .lineToLinearHeading(new Pose2d(-32.625, 0, Math.toRadians(90)))
                    .build();
            drive.followTrajectory(warehouse);

            if(robotRuntime.seconds() > 24){
                break;
            }
            i++;
        }
    }

    private void grabBlock(double speed){

    }

    private void dropBlock(){
        //Output Block
        intake.setPower(-0.3);
        sleep(2250);
        intake.setPower(0);
    }

    private void encoderArm(int level, int time){
        if(level == 0){
            arm_left.setTargetPosition(0);
            arm_right.setTargetPosition(0);
        }
        else if(level == 1){
            arm_left.setTargetPosition(400);
            arm_right.setTargetPosition(400);
        }
        else if(level == 2){
            arm_left.setTargetPosition(745);
            arm_right.setTargetPosition(745);
        }
        else{
            arm_left.setTargetPosition(1150);
            arm_right.setTargetPosition(1150);
        }

        arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_left.setPower(0.5);
        arm_right.setPower(0.5);
        armRuntime.reset();
        while ((arm_right.isBusy() || arm_left.isBusy()) || (armRuntime.seconds() <= time)){
            telemetry.addData("Runtime Seconds = ", armRuntime.seconds());
            telemetry.update();
        }
        arm_left.setPower(0.1);
        arm_right.setPower(0.1);
    }

    private void encoderArm(int level, double power){
        if(power != 0.0) {
            if (level == 0) {
                arm_left.setTargetPosition(0);
                arm_right.setTargetPosition(0);
            } else if (level == 1) {
                arm_left.setTargetPosition(400);
                arm_right.setTargetPosition(400);
            } else if (level == 2) {
                arm_left.setTargetPosition(745);
                arm_right.setTargetPosition(745);
            } else {
                arm_left.setTargetPosition(1050);
                arm_right.setTargetPosition(1050);
            }

            arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_left.setPower(power);
            arm_right.setPower(power);
        }
    }


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
     **/
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
