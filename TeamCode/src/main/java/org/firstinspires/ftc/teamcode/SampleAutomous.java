package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
/*
@Autonomous
public class SampleAutomous extends LinearOpMode
{
    Servo rightServo;
    Servo leftServo;
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ObjectSample camera = new ObjectSample(hardwareMap);
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d currentPose = new Pose2d();
        drive.setPoseEstimate(startPose);

        int distance = 0;



        if (!opModeIsActive()) {
            while (!opModeIsActive()) {
                if (camera.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = camera.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            if(recognition.getLabel().equals("")){
                                distance = 24;
                            }
                            else if(recognition.getLabel().equals("")){
                                distance = 48;
                            }
                            else{
                                distance = 0;
                            }

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("Distance: ", distance);
                        }
                        telemetry.update();
                    }
                }
            }
        }

        while(opModeIsActive())
        {
            MecanumDrive.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MecanumDrive.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            MecanumDrive.rightLift.setTargetPosition(0);
            MecanumDrive.leftLift.setTargetPosition(0);
            Trajectory beginningLineUp = drive.trajectoryBuilder(currentPose)
                    //Add temporal Marker to move arm up
                    .addTemporalMarker(0, () -> {
                        MecanumDrive.rightLift.setTargetPosition(1000);
                        MecanumDrive.leftLift.setTargetPosition(1000);
                    })

                    //Go past the cone
                    .splineToConstantHeading(new Vector2d(0, 24), Math.toRadians(90))
                    //Move left and go towards cone
                    .splineToLinearHeading(new Pose2d(-36, 24, Math.toRadians(104)), Math.toRadians(180))
                    //Line up to cone
                    .lineToLinearHeading(new Pose2d(-36, 27, Math.toRadians(104)))
                    .build();

            Trajectory dropCone = drive.trajectoryBuilder(currentPose)
                    //add temporal marker to move arm down
                    .addTemporalMarker(0, () -> {
                        MecanumDrive.rightLift.setTargetPosition(0);
                        MecanumDrive.leftLift.setTargetPosition(0);
                    })
                    //Move back
                    .lineToLinearHeading(new Pose2d(-36, 24, Math.toRadians(104)))
                    //Hopefully grab another cup
                    .splineToLinearHeading(new Pose2d(-36, 0, Math.toRadians(270)), Math.toRadians(270))
                    .build();

            Trajectory lineUp = drive.trajectoryBuilder(currentPose)
                    .splineToLinearHeading(new Pose2d(-36, 24, Math.toRadians(104)), Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(-36, 27, Math.toRadians(104)))
                    .build();

            Trajectory park = drive.trajectoryBuilder(currentPose)
                    .splineToLinearHeading(new Pose2d(-36, 24, Math.toRadians(90)), Math.toRadians(180))
                    .lineToLinearHeading(new Pose2d(distance, 27, Math.toRadians(90)))
                    .build();
            drive.followTrajectory(beginningLineUp);
            currentPose = drive.getPoseEstimate();
            //Servomove to drop cup
            leftServo.setPosition(1);
            rightServo.setPosition(0);

            //Drop Cup
            drive.followTrajectory(dropCone);
            currentPose = drive.getPoseEstimate();
            //Servomove to grab cup
            leftServo.setPosition(0);
            rightServo.setPosition(1);

            //Go to pole
            drive.followTrajectory(lineUp);
            currentPose = drive.getPoseEstimate();
            //servomove to drop cup
            leftServo.setPosition(1);
            rightServo.setPosition(0);

            break;
        }
    }

}
*/