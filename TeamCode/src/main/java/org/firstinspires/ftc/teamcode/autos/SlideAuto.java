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
    NanoClock clock = NanoClock.system();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private VoltageSensor ControlHub_VoltageSensor;
    public DcMotorEx rightLift;
    public DcMotorEx leftLift;
    public Servo leftServo;

    @Override
    public void runOpMode()
    {
        TelemetryPacket packet = new TelemetryPacket();

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
                    //lift up;
                })
                .addTemporalMarker(0.2, () -> {
                    //slide back;
                })
                .addTemporalMarker(0.8, () -> {
                    //lower lift;
                })
                .addTemporalMarker(1, () -> {
                    //claw drop;
                    //lift up
                })
                .addTemporalMarker(1.2, () -> {
                    //lift down to stack minus nth time
                    //open claw
                    //slide forward
                })
                .addTemporalMarker(1.5, () -> {
                    //close claw
                })
                .addTemporalMarker(1.6, () -> {
                    //raise to neutral
                })
                .addTemporalMarker(1.7, () -> {
                    //slide back to neutral
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
}
