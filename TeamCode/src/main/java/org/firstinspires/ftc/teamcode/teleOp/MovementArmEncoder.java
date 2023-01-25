package org.firstinspires.ftc.teamcode.teleOp;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.classes.MecanumDrive;

@TeleOp
@Config
public class MovementArmEncoder extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double maxSpeed = 1;
    public DcMotor rightLift;
    public DcMotor leftLift;
    public Servo leftServo;

    public void runOpMode() {
        mecanumDrive.init(hardwareMap);
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        //rightServo = hardwareMap.get(Servo.class, "rightServo");

        rightLift.setTargetPosition(0);
        leftLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY*/)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            //Gamepad 1 Code
            if(gamepad1.right_bumper){
                maxSpeed = 0.2;
                mecanumDrive.setMaxSpeed(maxSpeed);
                rotate = gamepad1.right_stick_x*0.4;
            }
            else{
                maxSpeed = 0.7;
                rotate = gamepad1.right_stick_x*0.8;
                mecanumDrive.setMaxSpeed(maxSpeed);
            }
            
            if(gamepad2.dpad_up)
            {
                if(gamepad2.left_bumper) {
                    rightLift.setTargetPosition(2000);
                    leftLift.setTargetPosition(2000);
                    leftLift.setPower(1);
                    rightLift.setPower(1);
                }else
                {
                    rightLift.setTargetPosition(2000);
                    leftLift.setTargetPosition(2000);
                    leftLift.setPower(powerFunction(leftLift.getCurrentPosition()));
                    rightLift.setPower(powerFunction(rightLift.getCurrentPosition()));
                }
            }
            else if(gamepad2.dpad_down)
            {
                rightLift.setTargetPosition(-2000);
                leftLift.setTargetPosition(-2000);
                leftLift.setPower(0.5);
                rightLift.setPower(0.5);
            }
            else{
                int num = (rightLift.getCurrentPosition() + leftLift.getCurrentPosition())/2;
                rightLift.setTargetPosition(num);
                leftLift.setTargetPosition(num);
            }

            if(gamepad2.a)
            {
                leftServo.setPosition(-1);
            }
            else
            {
                leftServo.setPosition(1);
            }

            mecanumDrive.driveMecanum(forward, strafe, rotate);

            telemetry.addData("Encoder Right Lift-*", rightLift.getCurrentPosition());
            telemetry.addData("Encoder Left Lift", leftLift.getCurrentPosition());
            telemetry.addData("Max Speed = ", maxSpeed);
            telemetry.update();
        }

    }
    public double powerFunction(int encoderTick)
    {
        double power = 0;
        if(encoderTick < 400)
        {
            power = 1;
        }
        else if(encoderTick < 1200)
        {
            power = 0.5;
        }
        else
        {
            power = 0.3;
        }
        return power;
    }
}
