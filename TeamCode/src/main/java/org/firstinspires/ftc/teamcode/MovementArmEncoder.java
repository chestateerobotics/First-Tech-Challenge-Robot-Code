package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class MovementArmEncoder extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double maxSpeed = 1;
    public DcMotor rightLift;
    public DcMotor leftLift;
    public Servo leftServo;
    public Servo rightServo;

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
        rightServo = hardwareMap.get(Servo.class, "rightServo");

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
            if(gamepad2.dpad_up)
            {
                rightLift.setTargetPosition(2150);
                leftLift.setTargetPosition(2150);
                leftLift.setPower(1);
                rightLift.setPower(1);

            }
            else if(gamepad2.dpad_down)
            {
                rightLift.setTargetPosition(0);
                leftLift.setTargetPosition(0);
                leftLift.setPower(0.8);
                rightLift.setPower(0.8);
            }
            else{
                int num = (rightLift.getCurrentPosition() + leftLift.getCurrentPosition())/2;
                rightLift.setTargetPosition(num);
                leftLift.setTargetPosition(num);
            }

            if(gamepad2.a)
            {
                leftServo.setPosition(-1);
                rightServo.setPosition(1);
            }
            else
            {
                leftServo.setPosition(1);
                rightServo.setPosition(-1);
            }

            mecanumDrive.driveMecanum(forward, strafe, rotate);
            telemetry.addData("Encoder Right Lift-*", rightLift.getCurrentPosition());
            telemetry.addData("Encoder Left Lift", leftLift.getCurrentPosition());
            telemetry.addData("Max Speed = ", maxSpeed);
            telemetry.update();
        }

    }
}
