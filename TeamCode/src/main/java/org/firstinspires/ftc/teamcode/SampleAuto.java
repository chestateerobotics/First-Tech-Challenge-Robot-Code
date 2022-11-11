package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;
import java.util.Objects;

@Autonomous
public class SampleAuto extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        WebcamClass camera = new WebcamClass(hardwareMap);

        while(!opModeIsActive() && camera.getTfod() != null)
        {
            List<Recognition> updatedRecognitions = camera.getTfod().getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                for (Recognition recognition : updatedRecognitions) {
                    if(Objects.equals(recognition.getLabel(), "cargill1"))
                        telemetry.addData("cargill for life", "");
                    else if(Objects.equals(recognition.getLabel(), "number2"))
                        telemetry.addData("number for life", "");
                    else
                        telemetry.addData("eagle for life", "");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                }
                telemetry.update();
            }
        }

        waitForStart();
        while(opModeIsActive())
        {

        }
    }
}
