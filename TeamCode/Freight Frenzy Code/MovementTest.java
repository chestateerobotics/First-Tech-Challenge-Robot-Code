package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class MovementTest extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private DcMotor arm_right;
    private DcMotor arm_left;
    private DcMotor intake;
    private DcMotor carousel;
    
    private boolean choice = false;
    private double maxRot = 0.8;
    private double maxSpeed = 1;
    private double armSpeed = 0.3;
    private double maxMotor = 1;
    private double power = 0.3;
    public void runOpMode() {
        mecanumDrive.init(hardwareMap);
        arm_right = hardwareMap.get(DcMotor.class, "arm_right");
        arm_left = hardwareMap.get(DcMotor.class, "arm_left");
        carousel = hardwareMap.get(DcMotor.class, "carousel_arm");
        intake = hardwareMap.get(DcMotor.class, "intake_arm");
        
        arm_right.setDirection(DcMotor.Direction.REVERSE);
        
        arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
            double strafe = gamepad1.left_stick_x;
            double rotate = maxRot * (gamepad1.right_stick_x);
/*
            if(gamepad1.a){
                choice = true;
            }
            else if(gamepad1.b){
                choice = false;
            }
*/          
            //Gamepad 1 Code
            if(gamepad1.right_trigger >= 0.5){
                maxSpeed = 1;
                mecanumDrive.setMaxSpeed(1);
            }
            else{
                maxSpeed = 0.7;
                mecanumDrive.setMaxSpeed(0.7);
            }
            if(choice){
                mecanumDrive.driveMecanum(-strafe, forward, rotate);
            }
            else{
                mecanumDrive.driveMecanum(forward, strafe, rotate);
            }
            
            
            
            //Gamepad 2 Code
            double up = 0.1 + gamepad2.left_stick_y * -1;
            arm_left.setPower(up * armSpeed);
            arm_right.setPower(up * armSpeed);
            if(gamepad2.y)
            {
                carousel.setPower(1);
                
            }
            else if(gamepad2.x)
            {
                carousel.setPower(0.6);
                
            }
            else if(gamepad2.b)
            {
                carousel.setPower(-1);
                
            }
            else if(gamepad2.a)
            {
                carousel.setPower(-0.6);
                
            }
            else{
                carousel.setPower(0);
            }
            
            if(gamepad2.right_trigger >= 0){
                carousel.setPower(-gamepad2.right_trigger);
            }
            if(gamepad2.left_trigger >= 0){
                carousel.setPower(gamepad2.left_trigger);
            }
            
            if(gamepad2.right_bumper){
                power = 0.3;
            }
            while(gamepad2.right_bumper){
               //Stop the robot to avoid bugs later on
               mecanumDrive.driveMecanum(0, 0, 0);
               arm_left.setPower(0.1);
               arm_right.setPower(0.1);
               
               
               //Ramp up while the button is pressed
               intake.setPower(-power);
               sleep(20);
               if(power <= 0.6){
                    power += 0.01;
               }
               telemetry.addData("Intake Speed = ", intake.getPower());
               telemetry.update();
            }
            if(gamepad2.left_bumper){
                intake.setPower(0.6);
            }
            else{
                intake.setPower(0);
            }
            power = 0;
            telemetry.addData("Choice = ", choice);
            telemetry.addData("Max Speed = ", maxSpeed);
            telemetry.addData("Intake Speed = ", intake.getPower());
            telemetry.addData("Carousel Speed = ", carousel.getPower());
            telemetry.update();
        }
    }
}
