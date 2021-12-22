package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.List;


@Autonomous

public class HardCode_and_DistanceSensor extends LinearOpMode{
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private DcMotor arm;
    private Servo handServo;
    private DistanceSensor rightDistance;

    // Game variables
    private String alliance = "Red";
    private int target;
    private double targetXMin, targetXMax;

    public void runOpMode() {
        //target = (int)(Math.random()*3)+1;
        target = 3;
        telemetry.addData("Target =", target);
        telemetry.update();
        Initiate();
        
        if (target == 1) {
            targetXMax = 60;
        } 
        else if (target == 2) {
            targetXMax = 100;
        } 
        else {
            targetXMax = 60;
        }
        
        waitForStart();
        while (opModeIsActive()) {
            if (target == 1) {
                moveForward(1);
                sleep(1300);
                fullStop();
                sleep(300);
                lineUp(0.25);
                sleep(300);
                moveArm();
                moveLeft(1);
                sleep(400);
                fullStop();
                sleep(300);
                break;
            }
            else if (target == 2){
                moveForward(1);
                sleep(1800);
                fullStop();
                sleep(400);
                lineUp(0.25);
                sleep(500);
                moveArm();
                moveLeft(1);
                sleep(400);
                fullStop();
                sleep(200);
                moveBackward(1);
                sleep(600);
                fullStop();
                sleep(300);
                break;
            }
            else if(target == 3){
                moveForward(1);
                sleep(2200);
                fullStop();
                sleep(500);
                lineUp(0.25);
                sleep(300);
                moveArm();
                sleep(500);
                moveLeft(1);
                sleep(400);
                fullStop();
                sleep(400);
                moveBackward(1);
                sleep(800);
                fullStop();
                sleep(400);
                break;
            }
        }
    }
    
    private void moveForward(double speed){
        mecanumDrive.driveMecanum(-speed, 0, 0);
    }
    
    private void moveBackward(double speed){
        mecanumDrive.driveMecanum(speed,0,0);
    }
    
    private void moveRight(double speed){
        mecanumDrive.driveMecanum(0,-speed,0);
    }
    
    private void moveLeft(double speed){
        mecanumDrive.driveMecanum(0,speed,0);
    }
    
    private void fullStop(){
        mecanumDrive.driveMecanum(0,0,0);
    }
    
    public void moveArm(){
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(-260);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.2);
        while (arm.isBusy()){
            telemetry.addData("Encoder Arm", arm.getCurrentPosition());
            telemetry.addData("Arm Power", arm.getPower());
            telemetry.update();
        }
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        handServo.setPosition(0);
        sleep(1200);
    }
    
    private void lineUp(double speed){
        while (rightDistance.getDistance(DistanceUnit.CM) < targetXMax) {
            moveLeft(speed);
        }
        fullStop();
    }
    
    private void Initiate(){
        mecanumDrive.init(hardwareMap);
        mecanumDrive.setMaxSpeed(0.4);
        mecanumDrive.setEndPowerBehaviorBrake();

        arm = hardwareMap.get(DcMotor.class, "arm_motor");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        handServo = hardwareMap.get(Servo.class, "arm_servo");

        rightDistance = hardwareMap.get(DistanceSensor.class, "RightDistanceSensor");;
    }
}
