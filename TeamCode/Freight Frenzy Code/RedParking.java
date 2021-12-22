package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@Autonomous

public class RedParking extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private DistanceSensor forward;
    private DistanceSensor right;
    private DcMotor arm_right;
    private DcMotor arm_left;
    private DcMotor intake;
    private DcMotor carousel;
    
    public void runOpMode() {
        initiate();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //Add Code
            moveArm(0.3);
            sleep(1000);
            moveArm(0.1);
            
            //Move Carousel
            carousel.setPower(0.5); //changed
            moveBack(0.2);
            sleep(450);
            moveBack(0.001);
            sleep(10000);
            fullStop();
            sleep(400);
            moveLeft(0.5);
            sleep(820);
            fullStop();
            sleep(500);
            
            /*
            moveLeft(0.5); //changed
            sleep(725);
            fullStop();
            sleep(580);
            moveBack(0.3);
            sleep(650);
            fullStop();
            sleep(300);
            */
            break;
        }
    }
    private void initiate(){
        mecanumDrive.init(hardwareMap);
        mecanumDrive.setEndPowerBehaviorBrake();
        forward = hardwareMap.get(DistanceSensor.class, "forward");
        right = hardwareMap.get(DistanceSensor.class, "right");
        arm_right = hardwareMap.get(DcMotor.class, "arm_right");
        arm_left = hardwareMap.get(DcMotor.class, "arm_left");
        carousel = hardwareMap.get(DcMotor.class, "carousel_arm");
        intake = hardwareMap.get(DcMotor.class, "intake_arm");
        
        arm_right.setDirection(DcMotor.Direction.REVERSE);
        
        arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    //mecanumDrive.driveMecanum(-strafe, forward, rotate);
    private void moveForward(double speed){
        mecanumDrive.driveMecanum(0, speed, 0);
    }
    private void moveForward(double speed, int distance){
        while(forward.getDistance(DistanceUnit.CM) < distance){
            moveForward(speed);
        }
        fullStop();
    }

    private void moveBack(double speed){
        mecanumDrive.driveMecanum(0, -speed, 0);
    }
    private void moveBack(double speed, int distance){
        while(forward.getDistance(DistanceUnit.CM) > distance){
            moveBack(speed);
        }
        fullStop();
    }
    
    private void moveLeft(double speed){
        mecanumDrive.driveMecanum(speed, 0, 0);
    }
    private void moveLeft(double speed, int distance){
        while(right.getDistance(DistanceUnit.CM) < distance){
            moveLeft(speed);
        }
        fullStop();
    }
    private void moveRight(double speed){
        mecanumDrive.driveMecanum(-speed, 0, 0);
    }
    private void moveRight(double speed, int distance){
        while(right.getDistance(DistanceUnit.CM) > distance){
            moveRight(speed);
        }
        fullStop();
    }
    
    private void fullStop(){
        mecanumDrive.driveMecanum(0,0,0);
    }

    
    private void moveArm(double speed){
        arm_left.setPower(speed);
        arm_right.setPower(speed);
    }
}
