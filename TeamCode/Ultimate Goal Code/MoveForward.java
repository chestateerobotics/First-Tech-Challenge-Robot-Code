package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous

public class MoveForward extends LinearOpMode{
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private DcMotor backLeft;
    private DcMotor backRight;
    private ColorSensor colorSensor;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Gyroscope imu;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor Motor3;
    private DcMotor Motor4;
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private CRServo intakeServo;
    private Servo wobbleServo;
    private DistanceSensor top;
    private DistanceSensor bottom;
    private int target;
    public ElapsedTime runtime = new ElapsedTime();

   
    
    // todo: write your code here
    
    
    
    public void runOpMode() {
        
        leftMotor = hardwareMap.dcMotor.get("LauncherFrontLeft");
        rightMotor = hardwareMap.dcMotor.get("LauncherFrontRight");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        Motor3 = hardwareMap.dcMotor.get("HighIntakeWheel");
        Motor4 = hardwareMap.dcMotor.get("LowIntakeWheel");
        top = hardwareMap.get(DistanceSensor.class, "top_sensor");
        bottom = hardwareMap.get(DistanceSensor.class, "bottom_sensor");

        
        mecanumDrive.init(hardwareMap);
        mecanumDrive.setMaxSpeed(0.4);
        target = 0;
        waitForStart();
        //runtime.reset();
        
        
        while (opModeIsActive()) {
            wobbleServo.setPosition(0.3);
            sleep(2000);
            driveForward(1, 1600);
            stopMovement();
            runtime.reset();
            while (runtime.seconds() <= 3){
                if(top.getDistance(DistanceUnit.CM) < 20){
                    
                target = 3;
                }
                else if(top.getDistance(DistanceUnit.CM) > 20 && bottom.getDistance(DistanceUnit.CM) < 10){
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
            strafeLeft(1, 80);
            stopMovement();
            setLauncher(-.354);
            sleep(1000);
            driveForward(1, 500);
            stopMovement();
            rotateClockwise(1, 36);
            stopMovement();
            setIntakeOne(1);
            setIntakeTwo (1);
            sleep(4000);
            rotateCounterClockwise(1, 40);
            stopMovement();
            
            if (target == 1){
            driveForward(1, 1000);
            stopMovement();
            strafeRight(1, 800);
            stopMovement();
            wobbleServo.setPosition(0.3);
            sleep(2000);
            }
            else if (target == 2){
            driveForward(1, 1900);
            stopMovement();
            strafeLeft(1, 300);
            stopMovement();
            sleep(1000);
            wobbleServo.setPosition(0.3);
            sleep(2000);
            strafeLeft(1, 500);
            stopMovement();
            sleep(1000);
            driveBackwards(1, 900);
            stopMovement();
            }
            else{
            driveForward(1, 2800);
            stopMovement();
            strafeRight(1, 900);
            stopMovement();
            sleep(1000);
            wobbleServo.setPosition(0.3);
            sleep(2000);
            strafeLeft(1, 500);
            stopMovement();
            sleep(1500);
            driveBackwards(1, 1700);
            stopMovement();
           }
            break;
            
        }
    }
    
    public void setLauncher(double speed) {
        rightMotor.setPower(-speed);
        leftMotor.setPower(speed);
    }
    
    public void stopMovement() {
        stopMovement(0);
    }
    
    public void stopMovement(int timeMs) {
        mecanumDrive.driveMecanum(0, 0, 0);
        sleep(timeMs);
    }
    
    public void driveForward(double speed, int timeMs) {
        mecanumDrive.driveMecanum(speed, 0, 0);
        sleep(timeMs);
    }
    
    public void driveBackwards(double speed, int timeMs) {
        mecanumDrive.driveMecanum(-speed, 0, 0);
        sleep(timeMs);
    }
    
    public void strafeLeft(double speed, int timeMs) {
        mecanumDrive.driveMecanum(0, -speed, 0);
        sleep(timeMs);
    }
    
    public void strafeRight(double speed, int timeMs) {
        mecanumDrive.driveMecanum(0, speed, 0);
        sleep(timeMs);
    }
    
    public void rotateClockwise(double speed, int timeMs) {
        mecanumDrive.driveMecanum(0, 0, speed);
        sleep(timeMs);
    }
    
    public void rotateCounterClockwise(double speed, int timeMs) {
        mecanumDrive.driveMecanum(0, 0, -speed);
        sleep(timeMs);
    }
    public void setIntakeOne(double speed) {
        Motor3.setPower(-speed);
        Motor3.setPower(speed);
    }
    public void setIntakeTwo(double speed) {
        Motor4.setPower(-speed);
        Motor4.setPower(speed);
    }
    public void Servo(double speed) {
        intakeServo.setPower(-speed);
        intakeServo.setPower(speed);
    }
    public void Servo1(double speed) {
        wobbleServo.setPosition(-speed);
        wobbleServo.setPosition(speed);
    }
}