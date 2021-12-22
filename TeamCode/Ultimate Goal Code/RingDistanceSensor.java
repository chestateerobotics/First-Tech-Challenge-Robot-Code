package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp

public class RingDistanceSensor extends LinearOpMode {
    private DistanceSensor top;
    private DistanceSensor bottom;
    private int target;
    //private ElapsedTime runtime;
    public void runOpMode() {
        top = hardwareMap.get(DistanceSensor.class, "top_sensor");
        bottom = hardwareMap.get(DistanceSensor.class, "bottom_sensor");
        target = 0;
        waitForStart();
        //runtime.reset();
        while(opModeIsActive()) {
            if(top.getDistance(DistanceUnit.CM) < 15){
                target = 3;
            }
            else if(top.getDistance(DistanceUnit.CM) > 15 && bottom.getDistance(DistanceUnit.CM) < 10){
                target = 2;
            }
            else{
                target = 1;
            }

            telemetry.addData("Target", target);
            telemetry.addData("Top Distance Sensor CM: ", top.getDistance(DistanceUnit.CM));
            telemetry.addData("Bottom Distance Sensor CM: ", bottom.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
