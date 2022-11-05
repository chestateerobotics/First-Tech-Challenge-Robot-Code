package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class Movement extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double maxSpeed = 1;

    public void runOpMode() {
        mecanumDrive.init(hardwareMap);
        int desiredValue = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            //Gamepad 1 Code
            if(gamepad1.right_trigger >= 0.5){
                maxSpeed = 0.1;
                mecanumDrive.setMaxSpeed(0.1);
            }
            else{
                maxSpeed = 0.5;
                mecanumDrive.setMaxSpeed(0.5);
            }
/*
            if(gamepad1.dpad_right){
                desiredValue = calc(20);
                mecanumDrive.leftLift.setTargetPosition(calc(20));
                mecanumDrive.rightLift.setTargetPosition(calc(20));
                mecanumDrive.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mecanumDrive.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mecanumDrive.leftLift.setPower(0.5);
                mecanumDrive.rightLift.setPower(0.5);
                telemetry.addData("Desired Encoder Value", calc(20));
            }
            else if(gamepad1.dpad_down){
                desiredValue = calc(10);
                mecanumDrive.leftLift.setTargetPosition(calc(10));
                mecanumDrive.rightLift.setTargetPosition(calc(10));
                mecanumDrive.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mecanumDrive.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mecanumDrive.leftLift.setPower(0.5);
                mecanumDrive.rightLift.setPower(0.5);
                telemetry.addData("Desired Encoder Value", calc(10));
            }
            else if(gamepad1.dpad_up){
                desiredValue = calc(30);
                mecanumDrive.leftLift.setTargetPosition(calc(30));
                mecanumDrive.rightLift.setTargetPosition(calc(30));
                mecanumDrive.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mecanumDrive.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mecanumDrive.rightLift.setPower(0.5);
                mecanumDrive.leftLift.setPower(0.5);
                telemetry.addData("Desired Encoder Value", calc(30));
            }
*/
            if(gamepad1.y)
            {
                mecanumDrive.rightLift.setPower(0.5);
                mecanumDrive.leftLift.setPower(0.5);
            }
            else if(gamepad1.a)
            {
                mecanumDrive.rightLift.setPower(-0.5);
                mecanumDrive.leftLift.setPower(-0.5);
            }
            else
            {
                mecanumDrive.rightLift.setPower(0);
                mecanumDrive.leftLift.setPower(0);
            }
            /*
            if(gamepad1.a)
            {
                mecanumDrive.leftServo.setPosition(-1);
                mecanumDrive.rightServo.setPosition(1);
            }
            else
            {
                mecanumDrive.leftServo.setPosition(1);
                mecanumDrive.rightServo.setPosition(-1);
            }
        */
            mecanumDrive.driveMecanum(forward, strafe, rotate);
            telemetry.addData("Encoder Right Lift", mecanumDrive.rightLift.getCurrentPosition());
            telemetry.addData("Encoder Left Lift", mecanumDrive.leftLift.getCurrentPosition());
            telemetry.addData("Desired Encoder Value", desiredValue);
            telemetry.addData("Max Speed = ", maxSpeed);
            telemetry.update();
        }

    }
    private int calc(int inch)
    {
        int encoderValue = (int)((((inch/6)*25.4)/8)*145.1);
        return encoderValue;
    }


}
