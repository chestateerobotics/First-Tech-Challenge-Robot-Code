package org.firstinspires.ftc.teamcode;

import java.util.List;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//Program parks in the squares without sensors 

@Autonomous

public class TestPowerShotAuto extends LinearOpMode{
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private DcMotor arm;
    private Servo handServo;
    private DistanceSensor rightDistance, frontDistance, backDistance, leftDistance;
    private DcMotor leftLauncher, rightLauncher;
    private CRServo topRoller;
    private DcMotor bottomRoller;
    private int target;

    //setLauncher(.354);

    public void runOpMode(){
        telemetry.addData("Ready to start", "");
        telemetry.update();
        Initiate();
        target = 1;
        
        waitForStart();
        while (opModeIsActive()){
            sleep(1000);
            moveBackward(1);
            sleep(400);
            fullStop();
            sleep(800);
            //Line up with the first Power Shot
            while(leftDistance.getDistance(DistanceUnit.CM) > 81){
                moveLeft(0.5);
            }
            fullStop();
            sleep(500);
            while(leftDistance.getDistance(DistanceUnit.CM) <= 78){
                moveRight(0.4);
            }
            fullStop();
            sleep(500);
            //Shoot Power Shot and continue down the line
            setLauncher(0.344);
            sleep(500);

            //Find the right timing to stop
            for(int x = 0; x < 3; x++) {
                setIntakeBottom(0.5);
                setIntakeTop(1);
                sleep(2000);
                setIntakeBottom(0);
                setIntakeTop(0);

                //Move left
                moveLeft(0.8);
                sleep(600);
                fullStop();
                sleep(1000);
            }
            moveForward(1);
            sleep(300);
            fullStop();
            
            break;
            
        }
    }
    
    private void moveForward(double speed){
        mecanumDrive.driveMecanum(speed, 0, 0);
    }
    
    private void moveBackward(double speed){
        mecanumDrive.driveMecanum(-speed,0,0);
    }
    
    private void moveRight(double speed){
        mecanumDrive.driveMecanum(0,speed,0);
    }
    
    private void moveLeft(double speed){
        mecanumDrive.driveMecanum(0,-speed,0);
    }
    
    private void fullStop(){
        mecanumDrive.driveMecanum(0,0,0);
    }
    
    private void Initiate(){
        mecanumDrive.init(hardwareMap);
        mecanumDrive.setMaxSpeed(0.4);
        mecanumDrive.setEndPowerBehaviorBrake();

        leftLauncher = hardwareMap.dcMotor.get("LauncherFrontLeft");
        rightLauncher = hardwareMap.dcMotor.get("LauncherFrontRight");

        topRoller = hardwareMap.get(CRServo.class, "HighIntakeWheel");
        bottomRoller = hardwareMap.dcMotor.get("LowIntakeWheel");

        arm = hardwareMap.get(DcMotor.class, "arm_motor");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        handServo = hardwareMap.get(Servo.class, "arm_servo");

        rightDistance = hardwareMap.get(DistanceSensor.class, "RightDistanceSensor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "FrontDistanceSensor");
        leftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistanceSensor");
        backDistance = hardwareMap.get(DistanceSensor.class, "BackDistanceSensor");
    }

    private void setLauncher(double speed) {
        rightLauncher.setPower(speed);
        leftLauncher.setPower(-speed);
    }

    private void setIntakeBottom(double speed) {
        //bottomRoller.setPower(-speed);
        bottomRoller.setPower(speed);
    }

    private void setIntakeTop(double speed) {
        //topRoller.setPower(-speed);
        topRoller.setPower(speed);
    }
}
