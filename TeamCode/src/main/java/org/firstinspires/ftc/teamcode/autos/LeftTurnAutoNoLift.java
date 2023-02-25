package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.classes.WebcamClass;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;
import java.util.Objects;

@Config
@Autonomous
public class LeftTurnAutoNoLift extends LinearOpMode
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
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private VoltageSensor ControlHub_VoltageSensor;
    //public Servo rightServo;
    public int encoderSubtracter = 0;
    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
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
                .waitSeconds(0.25)
                .forward(MOVE_FORWARD, SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(TURN_ALIGN), 2, Math.toRadians(184.02607784577722))
                .forward(6, SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED-10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence backLower = drive.trajectorySequenceBuilder(startMove.end())
                .waitSeconds(0.5)
                .back(5)
                .lineToLinearHeading(new Pose2d(0, 50, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(19, SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence backLower2 = drive.trajectorySequenceBuilder(startMove.end())
                .waitSeconds(0.5)
                .back(5)
                .lineToLinearHeading(new Pose2d(0, 50, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(19, SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence back1 = drive.trajectorySequenceBuilder(backLower.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(0,MOVE_FORWARD, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(135), 2, Math.toRadians(184.02607784577722))
                .forward(4)
                .build();
        TrajectorySequence back2 = drive.trajectorySequenceBuilder(backLower.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(0,MOVE_FORWARD, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(135), 2, Math.toRadians(184.02607784577722))
                .forward(4.5, SampleMecanumDrive.getVelocityConstraint(CYCLE_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(back1.end())
                .waitSeconds(0.5)
                .turn(Math.toRadians(-40),2, Math.toRadians(184.02607784577722))
                .lineToConstantHeading(new Vector2d(-1, 48))
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(parkMiddle.end())
                .lineToConstantHeading(new Vector2d(-25, 48))
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(parkMiddle.end())
                .lineToConstantHeading(new Vector2d(23, 47))
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
                    packet.put("Detected Image:",  recognition.getLabel() + "Confidence Level: " + recognition.getConfidence() * 100);
                    packet.put("Voltage:", ControlHub_VoltageSensor.getVoltage());
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                }
                dashboard.sendTelemetryPacket(packet);
                telemetry.update();
            }
        }

        waitForStart();
        while (opModeIsActive()) {
            drive.followTrajectorySequence(startMove);
            drive.followTrajectorySequence(backLower);
            drive.followTrajectorySequence(back1);
            drive.followTrajectorySequence(backLower);
            drive.followTrajectorySequence(back2);
            if(objDetect.equals("eagle3"))
            {
                drive.followTrajectorySequence(parkMiddle);
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
            leftServo.setPosition(-1);
            packet.put("Right Power", rightLift.getPower());
            packet.put("Left Power", leftLift.getPower());
            packet.put("Right Position", rightLift.getCurrentPosition());
            packet.put("Left Position", leftLift.getCurrentPosition());
            packet.put("Voltage", ControlHub_VoltageSensor.getVoltage());
            telemetry.addData("Current Pose", startMove.end());
            telemetry.addData("obj detected", objDetect);
            packet.put("Object detected", objDetect);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

            break;
        }

    }
}