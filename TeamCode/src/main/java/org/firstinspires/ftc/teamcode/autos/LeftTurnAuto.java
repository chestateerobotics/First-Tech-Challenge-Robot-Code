package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.classes.WebcamClass;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.List;
import java.util.Objects;

@Config
@Autonomous
public class LeftTurnAuto extends LinearOpMode
{
    String objDetect = "";
    public static double MOVE_FORWARD = 52.0;
    public static double TURN_ALIGN = 45;
    public static int HIGH_VALUE = 1600;
    public static int MIDDLE_VALUE = 1200;
    public static int LOW_VALUE = 1000;
    public static double FORWARD_ALIGN = 3;
    public DcMotor rightLift;
    public DcMotor leftLift;
    public Servo leftServo;
    public Servo rightServo;
    @Override
    public void runOpMode() {
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        rightLift.setTargetPosition(0);
        leftLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        WebcamClass camera = new WebcamClass(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        leftServo.setPosition(1);
        rightServo.setPosition(-1);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        Trajectory startMove = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .forward(MOVE_FORWARD)
                .addTemporalMarker(0, () -> {
                    // Turn on motor
                    encoderArm(3, 0.5);
                    })
                .addTemporalMarker(2.5, () -> {
                    // Turn on motor
                    encoderArm(0, 0);
                })
                .build();
        Trajectory backLower = drive.trajectoryBuilder(startMove.end().plus(new Pose2d(0,0, Math.toRadians(TURN_ALIGN))))
                .lineToLinearHeading(new Pose2d(0, MOVE_FORWARD-2, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, () -> {
                    // Turn on motor
                    leftServo.setPosition(-1);
                    rightServo.setPosition(1);
                    encoderArm(4, 0.5);
                })
                .addTemporalMarker(2.0, () -> {
                    // Turn on motor
                    encoderArm(0, 0);
                })
                .build();
        Trajectory forward2 = drive.trajectoryBuilder(backLower.end())
                .forward(22, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory back1 = drive.trajectoryBuilder(forward2.end())
                .forward(-22)
                .build();
        Trajectory align = drive.trajectoryBuilder(back1.end())
                .lineToLinearHeading(new Pose2d(0,MOVE_FORWARD, Math.toRadians(135)))
                .build();
        while (!opModeIsActive() && camera.getTfod() != null) {
            List<Recognition> updatedRecognitions = camera.getTfod().getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                for (Recognition recognition : updatedRecognitions) {
                    if (Objects.equals(recognition.getLabel(), "cargill1"))
                        objDetect = "cargill1";
                    else if (Objects.equals(recognition.getLabel(), "number2"))
                        objDetect = "number2";
                    else
                        objDetect = "eagle3";
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                }
                telemetry.update();
            }
        }

        waitForStart();
        while (opModeIsActive()) {
            drive.followTrajectory(startMove);
            drive.turn(Math.toRadians(TURN_ALIGN));
            //drive.followTrajectory(forward);
            //drive.followTrajectory(back);
            drive.followTrajectory(backLower);
            drive.followTrajectory(forward2);
            drive.followTrajectory(align);
            leftServo.setPosition(1);
            rightServo.setPosition(-1);
            drive.followTrajectory(back1);
            telemetry.addData("Current Pose", startMove.end());
            telemetry.addData("obj detected", objDetect);
            telemetry.update();

            break;
        }

    }
    private void encoderArm(int level, double power){
        if(power != 0.0) {
            if (level == 0) {
                leftLift.setTargetPosition(0);
                rightLift.setTargetPosition(0);
            } else if (level == 1) {
                rightLift.setTargetPosition(LOW_VALUE);
                leftLift.setTargetPosition(LOW_VALUE);
            } else if (level == 2) {
                rightLift.setTargetPosition(MIDDLE_VALUE);
                leftLift.setTargetPosition(MIDDLE_VALUE);
            } else if(level == 3) {
                rightLift.setTargetPosition(HIGH_VALUE);
                leftLift.setTargetPosition(HIGH_VALUE);
            }
            else
            {
                rightLift.setTargetPosition(350);
                leftLift.setTargetPosition(350);
            }

            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(power);
            leftLift.setPower(power);
        }
    }
}
