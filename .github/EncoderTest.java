package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Encoder Test")

public class EncoderTest extends LinearOpMode{
    private MecanumDrive mecanumDrive = new MecanumDrive();

    public void runOpMode() {
        mecanumDrive.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            for(int x = 0; x < 10; x++){
                moveForward(30,1);
                moveLeft(10,1);
                moveRight(10,1);
                moveBackward(30,1);
            }

        }


    }
    public void moveForward(int distance, int speed){
        telemetry.addData("Going forward", "");
        telemetry.update();
        double cent = mecanumDrive.getDistanceCm()[0] + distance;
        while(mecanumDrive.getDistanceCm()[0] < cent){
            mecanumDrive.driveMecanum(speed,0,0);
        }
    }
    public void moveBackward(int distance, int speed){
        telemetry.addData("Going Back", "");
        telemetry.update();
        double cent = mecanumDrive.getDistanceCm()[0] - distance;
        while(mecanumDrive.getDistanceCm()[0] > cent){
            mecanumDrive.driveMecanum(-speed,0,0);
        }
    }
    public void moveLeft(int distance, int speed){
        telemetry.addData("Going Left", "");
        telemetry.update();
        double cent = mecanumDrive.getDistanceCm()[1] - distance;
        while(mecanumDrive.getDistanceCm()[1] > cent){
            mecanumDrive.driveMecanum(0,-speed,0);
        }
    }
    public void moveRight(int distance, int speed){
        telemetry.addData("Going Right", "");
        telemetry.update();
        double cent = mecanumDrive.getDistanceCm()[1] + distance;
        while(mecanumDrive.getDistanceCm()[1] < cent){
            mecanumDrive.driveMecanum(0,speed,0);
        }
    }
}