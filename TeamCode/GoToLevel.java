package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@Autonomous
public class GoToLevel extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private DistanceSensor forward;
    private DistanceSensor right;
    private DcMotor arm_right;
    private DcMotor arm_left;
    private DcMotor intake;
    private DcMotor carousel;
    private int encoder;

    public void runOpMode() {
        initiate();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            encoderArm(3);
            sleep(300);
            //Use Encoders to go to the hub
            moveForward(0.5);
            sleep(300);
            //Move while encoder isn't at the right distance
            double num = calculateEncoders(609.6);
            while(intake.getCurrentPosition() <= num){
                moveForward(0.1);
            }
            fullStop();
            /*
            intake.setPower(-1);
            sleep(2000);
            intake.setPower(0);
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

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    private void encoderArm(int level){
        if(level == 1){
            arm_left.setTargetPosition(250);
            arm_right.setTargetPosition(250);
        }
        else if(level == 2){
            arm_left.setTargetPosition(500);
            arm_right.setTargetPosition(500);
        }
        else{
            arm_left.setTargetPosition(850);
            arm_right.setTargetPosition(850);
        }

        arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_left.setPower(0.25);
        arm_right.setPower(0.25);
        while (arm_right.isBusy() && arm_left.isBusy()){
        }
        arm_left.setPower(0);
        arm_right.setPower(0);
    }

    //Returns the Encoder values needed to run the motors
    private double calculateEncoders(double distance){
        int diameter = 35; //In mm
        double encoders = 8192;
        double revolutions = distance / (diameter * Math.PI);
        return revolutions * encoders;
    }
}
