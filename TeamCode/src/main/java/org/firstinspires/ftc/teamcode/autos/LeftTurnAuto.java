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
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;
import java.util.Objects;

@Config
@Autonomous
public class LeftTurnAuto extends LinearOpMode
{
    String objDetect = "";
    public static double MOVE_FORWARD = 52.5;
    public static double TURN_ALIGN = 45;
    public static int HIGH_VALUE = 1900;
    public static int MIDDLE_VALUE = 1200;
    public static int LOW_VALUE = 1000;
    public static int CUP_VALUE = 350;
    public static double CYCLE_SPEED = 40;
    public static double FORWARD_ALIGN = 3;
    public DcMotor rightLift;
    public DcMotor leftLift;
    public Servo leftServo;
    //public Servo rightServo;
    public int encoderSubtracter = 0;
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
        //rightServo = hardwareMap.get(Servo.class, "rightServo");

        rightLift.setTargetPosition(0);
        leftLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        WebcamClass camera = new WebcamClass(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        TrajectorySequence startMove = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .waitSeconds(1)
                .forward(MOVE_FORWARD, SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1, () -> {
                    // Turn on motor
                    encoderArm(3, 0.6);
                })
                .addTemporalMarker(1.5, () -> {
                    // Slowdown
                    encoderArm(3, 0.2);
                })
                .turn(Math.toRadians(TURN_ALIGN), 2, Math.toRadians(184.02607784577722))
                .forward(7, SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED-10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence backLower = drive.trajectorySequenceBuilder(startMove.end())
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(0, () -> {
                    // Turn on motor
                    encoderArm(5, 0.6);
                })
                .addTemporalMarker(0.5, () -> {
                    // Turn on motor
                    encoderArm(0, 0);
                })
                .lineToLinearHeading(new Pose2d(0, 50, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .addTemporalMarker(0.5, () -> {
                    // Turn on motor
                    leftServo.setPosition(-1);
                })
                .addTemporalMarker(1.0, () -> {
                    // Turn on motor
                    encoderArm(4, 0.5);
                })
                .forward(19, SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence backLower2 = drive.trajectorySequenceBuilder(startMove.end())
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(0, () -> {
                    // Turn on motor
                    encoderArm(5, 0.6);
                })
                .addTemporalMarker(0.5, () -> {
                    // Turn on motor
                    encoderArm(0, 0);
                })
                .lineToLinearHeading(new Pose2d(0, 50, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .addTemporalMarker(0.5, () -> {
                    // Turn on motor
                    leftServo.setPosition(-1);
                })
                .addTemporalMarker(1.0, () -> {
                    // Turn on motor
                    encoderArm(4, 0.5);
                })
                .forward(18.5, SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence back1 = drive.trajectorySequenceBuilder(backLower.end())
                .addTemporalMarker(0, () -> {
                    leftServo.setPosition(1);
                })
                .waitSeconds(1)
                .addTemporalMarker(0.5, () -> {
                    // Turn on motor
                    encoderArm(3, 0.6);
                })
                .addTemporalMarker(1.5, () -> {
                    // Slowdown
                    encoderArm(3, 0.2);
                })
                .lineToLinearHeading(new Pose2d(0,MOVE_FORWARD, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(135), 2, Math.toRadians(184.02607784577722))
                .forward(6)
                .build();
        TrajectorySequence back2 = drive.trajectorySequenceBuilder(backLower.end())
                .addTemporalMarker(0, () -> {
                    leftServo.setPosition(1);
                })
                .waitSeconds(1)
                .addTemporalMarker(0.5, () -> {
                    // Turn on motor
                    encoderArm(3, 0.6);
                })
                .addTemporalMarker(1.5, () -> {
                    // Slowdown
                    encoderArm(3, 0.2);
                })
                .lineToLinearHeading(new Pose2d(0,MOVE_FORWARD, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(135), 2, Math.toRadians(184.02607784577722))
                .forward(6, SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(back1.end())
                .waitSeconds(1.5)
                .addTemporalMarker(0, () -> {
                    // Turn on motor
                    encoderArm(5, 0.6);
                })
                .addTemporalMarker(1.5, () -> {
                    // Turn on motor
                    leftServo.setPosition(-1);
                    encoderArm(0, 0.7);
                })
                .addTemporalMarker(4.5, () -> {
                    // Turn on motor
                    encoderArm(0, 0);
                })
                .turn(Math.toRadians(-40),2, Math.toRadians(184.02607784577722))
                .lineToConstantHeading(new Vector2d(-1, 48))
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(parkMiddle.end())
                .lineToConstantHeading(new Vector2d(-25, 48))
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(parkMiddle.end())
                .strafeRight(25)
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
            leftServo.setPosition(1);
            drive.followTrajectorySequence(startMove);
            drive.followTrajectorySequence(backLower);
            encoderSubtracter+=25;
            drive.followTrajectorySequence(back1);
            drive.followTrajectorySequence(backLower);
            encoderSubtracter+=25;
            drive.followTrajectorySequence(back2);
            leftServo.setPosition(-1);
            if(objDetect.equals("eagle3"))
            {
                drive.followTrajectorySequence(parkMiddle);
                drive.followTrajectorySequence(parkRight);
                leftServo.setPosition(-1);
            }else if(objDetect.equals("cargill1"))
            {
                drive.followTrajectorySequence(parkMiddle);
                drive.followTrajectorySequence(parkLeft);
                leftServo.setPosition(-1);
            }
            else
            {
                drive.followTrajectorySequence(parkMiddle);
                leftServo.setPosition(-1);
            }
            leftServo.setPosition(-1);

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
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (level == 1) {
                rightLift.setTargetPosition(LOW_VALUE);
                leftLift.setTargetPosition(LOW_VALUE);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (level == 2) {
                rightLift.setTargetPosition(MIDDLE_VALUE);
                leftLift.setTargetPosition(MIDDLE_VALUE);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (level == 3) {
                rightLift.setTargetPosition(HIGH_VALUE);
                leftLift.setTargetPosition(HIGH_VALUE);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if(level == 5){
                rightLift.setTargetPosition(rightLift.getCurrentPosition()-300);
                leftLift.setTargetPosition(rightLift.getCurrentPosition()-300);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else {
                rightLift.setTargetPosition(300-encoderSubtracter);
                leftLift.setTargetPosition(300-encoderSubtracter);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

        }
        rightLift.setPower(power);
        leftLift.setPower(power);

    }
}