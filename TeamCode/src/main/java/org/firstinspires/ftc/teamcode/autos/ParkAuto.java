package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.classes.WebcamClass;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;
import java.util.Objects;

@Config
@Autonomous
public class ParkAuto extends LinearOpMode
{
    String objDetect = "";
    @Override
    public void runOpMode() {
        WebcamClass camera = new WebcamClass(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(0, 26))
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(parkMiddle.end())
                .strafeLeft(20)
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(parkMiddle.end())
                .strafeLeft(-20)
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
        while (opModeIsActive())
        {
            if(objDetect.equals("cargill1"))
            {
                drive.followTrajectorySequence(parkMiddle);
                drive.followTrajectorySequence(parkRight);
            }else if(objDetect.equals("eagle3"))
            {
                drive.followTrajectorySequence(parkMiddle);
                drive.followTrajectorySequence(parkLeft);
            }
            else
            {
                drive.followTrajectorySequence(parkMiddle);
            }
            telemetry.addData("Current Pose", parkMiddle.end());
            telemetry.addData("obj detected", objDetect);
            telemetry.update();

            break;
        }
    }
}
