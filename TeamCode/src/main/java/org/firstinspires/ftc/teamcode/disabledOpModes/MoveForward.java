package org.firstinspires.ftc.teamcode.disabledOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.MecanumDrive;

@Disabled
@Autonomous
public class MoveForward extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    public void runOpMode(){
        mecanumDrive.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            mecanumDrive.driveMecanum(0.5,0,0);
            sleep(1500);
            mecanumDrive.driveMecanum(0,0,0);
            break;
        }
    }
}