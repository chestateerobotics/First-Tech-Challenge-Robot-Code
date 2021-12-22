package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class DistanceSensing extends LinearOpMode {
    private DistanceSensor front;
    private DistanceSensor right;
    private DistanceSensor left;
    private DistanceSensor back;
    
    public void runOpMode() {
        right = hardwareMap.get(DistanceSensor.class, "RightDistanceSensor");
        front = hardwareMap.get(DistanceSensor.class, "FrontDistanceSensor");
        left = hardwareMap.get(DistanceSensor.class, "LeftDistanceSensor");
        back = hardwareMap.get(DistanceSensor.class, "BackDistanceSensor");
        
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Front range", String.format("%.01f cm", front.getDistance(DistanceUnit.CM)));
            telemetry.addData("Back range", String.format("%.01f cm", back.getDistance(DistanceUnit.CM)));
            telemetry.addData("Left range", String.format("%.01f cm", left.getDistance(DistanceUnit.CM)));
            telemetry.addData("Right range", String.format("%.01f cm", right.getDistance(DistanceUnit.CM)));

            telemetry.update();
        }
    }

}