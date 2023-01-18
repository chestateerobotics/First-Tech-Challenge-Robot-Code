package org.firstinspires.ftc.teamcode.teleOp;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.classes.MecanumDrive;

@TeleOp
@Config
public class LiftFunctionTester extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    public DcMotor rightLift;
    public DcMotor leftLift;
    //public Servo rightServo;

    public void runOpMode() {
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
                    rightLift.setPower(powerFunction(leftLift.getCurrentPosition()));
                }
            }
            else if(gamepad2.dpad_down)
            {
                rightLift.setTargetPosition(-2000);
                leftLift.setTargetPosition(-2000);
                leftLift.setPower(0.4);
                rightLift.setPower(0.4);
            }
            else{
                int num = (rightLift.getCurrentPosition() + leftLift.getCurrentPosition())/2;
                rightLift.setTargetPosition(num);
                leftLift.setTargetPosition(num);
            }


            telemetry.addData("Encoder Right Lift-*", rightLift.getCurrentPosition());
            telemetry.addData("Encoder Left Lift", leftLift.getCurrentPosition());
            telemetry.addData("Right Lift Power = ", rightLift.getPower());
            telemetry.addData("Left Lift Power = ", leftLift.getPower());
            telemetry.update();
        }

    }

    public double powerFunction(int encoderTick)
    {
        double power = 0;
        if(encoderTick < 800)
        {
            power = 1;
        }
        else if(encoderTick < 1200)
        {
            power = 0.4;
        }
        else
        {
            power = 0.6;
        }
        return power;
    }

}
