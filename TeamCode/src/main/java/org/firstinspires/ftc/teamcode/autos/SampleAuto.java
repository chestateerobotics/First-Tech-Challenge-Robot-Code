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

import java.util.List;
import java.util.Objects;

@Config
@Autonomous
public class SampleAuto extends LinearOpMode
{
    String objDetect = "";
    public static double MOVE_FORWARD = 50.0;
    public static double TURN_ALIGN = 90;
    public static double BACK_ALIGN = 13;
    public static double FINAL_ANGLE = -15;
    @Override
    public void runOpMode()
    {
        WebcamClass camera = new WebcamClass(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        Trajectory startMove = drive.trajectoryBuilder(new Pose2d(0,0, Math.toRadians(90)))
                .forward(MOVE_FORWARD)
                .build();
        Trajectory lineUpBack = drive.trajectoryBuilder(startMove.end().plus(new Pose2d(0,0, Math.toRadians(TURN_ALIGN))))
                .strafeTo(new Vector2d(BACK_ALIGN, MOVE_FORWARD))
                .build();
        Trajectory moveLeft = drive.trajectoryBuilder(lineUpBack.end().plus(new Pose2d(0, 0, Math.toRadians(FINAL_ANGLE))))
                .strafeTo(new Vector2d(BACK_ALIGN, MOVE_FORWARD+4))
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
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                }
                telemetry.update();
            }
        }

        waitForStart();
        while(opModeIsActive())
        {
            drive.followTrajectory(startMove);
            drive.turn(Math.toRadians(TURN_ALIGN));
            drive.followTrajectory(lineUpBack);
            drive.turn(Math.toRadians(FINAL_ANGLE));
            drive.followTrajectory(moveLeft);
            telemetry.addData("Current Pose", startMove.end());
            telemetry.addData("obj detected", objDetect);
            telemetry.update();

            break;
        }
    }
}
