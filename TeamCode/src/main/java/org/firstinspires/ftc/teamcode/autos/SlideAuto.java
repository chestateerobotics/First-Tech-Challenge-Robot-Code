package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.classes.WebcamClass;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;
import java.util.Objects;

@Config
@Autonomous
public class SlideAuto extends LinearOpMode
{
    String objDetect = "";
    public static double MOVE_FORWARD = 50.0;
    public static double TURN_ALIGN = -90;
    public static double BACK_ALIGN = 9;
    public static double FINAL_ANGLE = -15;
    public static double TUNE_LEFT = 7.5;

    public static int HIGH_VALUE = 1900;
    public static int MIDDLE_VALUE = 1200;
    public static int LOW_VALUE = 1000;

    public static int SLIDE_HIGH = -1500;
    public static int SLIDE_GRAB = 1000;
    public static int SLIDE_NEUTRAL = 0;

    public int encoderSubtracter = 0;

    NanoClock clock = NanoClock.system();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private VoltageSensor ControlHub_VoltageSensor;
    public DcMotorEx rightLift;
    public DcMotorEx leftLift;
    public DcMotorEx slideLift;
    public Servo leftServo;

    @Override
    public void runOpMode()
    {
        TelemetryPacket packet = new TelemetryPacket();

        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        rightLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        leftLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideLift = hardwareMap.get(DcMotorEx.class, "slideLift");
        slideLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        //rightServo = hardwareMap.get(Servo.class, "rightServo");

        rightLift.setTargetPosition(0);
        leftLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLift.setTargetPosition(0);
        slideLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dashboard.setTelemetryTransmissionInterval(25);
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        WebcamClass camera = new WebcamClass(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        TrajectorySequence startMove = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(90)))
                .forward(MOVE_FORWARD)
                .turn(Math.toRadians(TURN_ALIGN))
                .strafeTo(new Vector2d(BACK_ALIGN, MOVE_FORWARD))
                .turn(Math.toRadians(FINAL_ANGLE))
                .strafeLeft(TUNE_LEFT)
                .build();
        TrajectorySequence CYCLE_1 = drive.trajectorySequenceBuilder(startMove.end())
                .addTemporalMarker(0, () -> {
                    //slide and arm to high junction
                    encoderArm(3, 0.8);
                    encoderSlide(1, 0.5);
                })
                .addTemporalMarker(0.5, () -> {
                    //lower lift;
                    encoderArm(5, 0.3);
                })
                .addTemporalMarker(0.6, () -> {
                    //claw drop;
                    leftServo.setPosition(-1);
                })
                .addTemporalMarker(0.7, () -> {
                    //lift up
                    encoderArm(3, 0.3);
                })
                .addTemporalMarker(0.8, () -> {
                    //slide forward
                    encoderSlide(2, 1);
                })
                .addTemporalMarker(0.85, () -> {
                    //raise down
                    encoderArm(4, 0.7);
                })
                .addTemporalMarker(1.3, () -> {
                    //close claw
                    leftServo.setPosition(1);
                })
                .addTemporalMarker(1.4, () -> {
                    //raise up to neutral
                    encoderArm(1, 0.7);
                })
                .addTemporalMarker(1.5, () -> {
                    //raise up to neutral
                    encoderSlide(1, 0.7);
                })
                .build();
        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(startMove.end())
                .lineToLinearHeading(new Pose2d(-1, 48, Math.toRadians(90)))
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(parkMiddle.end())
                .lineToLinearHeading(new Pose2d(-25, 48, Math.toRadians(90)))
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(startMove.end())
                .lineToLinearHeading(new Pose2d(20, 47, Math.toRadians(90)))
                .build();

        while(!opModeIsActive() && camera.getTfod() != null)
        {
            List<Recognition> updatedRecognitions = camera.getTfod().getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                for (Recognition recognition : updatedRecognitions) {
                    if(Objects.equals(recognition.getLabel(), "cargill1"))
                        objDetect = "cargill1";
                    else if(Objects.equals(recognition.getLabel(), "number2"))
                        objDetect = "number2";
                    else
                        objDetect = "eagle3";
                    packet.put("Detected Image:",  recognition.getLabel() + "Confidence Level: " + recognition.getConfidence() * 100);
                    packet.put("Voltage:", ControlHub_VoltageSensor.getVoltage());
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                }
                dashboard.sendTelemetryPacket(packet);
                telemetry.update();
            }
        }

        waitForStart();
        double profileStart = clock.seconds();
        while(opModeIsActive())
        {
            double profileTime = clock.seconds() - profileStart;
            drive.followTrajectorySequence(startMove);
            while (profileTime < 25)
            {
                drive.followTrajectorySequence(CYCLE_1);
                profileTime = clock.seconds() - profileStart;
            }
            if(objDetect.equals("eagle3"))
            {
                drive.followTrajectorySequence(parkRight);
            }else if(objDetect.equals("cargill1"))
            {
                drive.followTrajectorySequence(parkMiddle);
                drive.followTrajectorySequence(parkLeft);
            }
            else
            {
                drive.followTrajectorySequence(parkMiddle);
            }
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
                rightLift.setTargetPosition(rightLift.getCurrentPosition()-500);
                leftLift.setTargetPosition(rightLift.getCurrentPosition()-500);
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

    private void encoderSlide(int level, double power){
        if(power != 0.0) {
            if (level == 0) {
                slideLift.setTargetPosition(SLIDE_NEUTRAL);
                slideLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (level == 1) {
                slideLift.setTargetPosition(SLIDE_HIGH);
                slideLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (level == 2) {
                slideLift.setTargetPosition(SLIDE_GRAB);
                slideLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        slideLift.setPower(power);

    }

}
