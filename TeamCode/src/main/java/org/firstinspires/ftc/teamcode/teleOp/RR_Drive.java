package org.firstinspires.ftc.teamcode.teleOp;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autos.PoseStorage;
import org.firstinspires.ftc.teamcode.classes.PIDController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp
@Config
public class RR_Drive extends LinearOpMode {
    private double maxSpeed = 1;
    private int slideValue =0;
    public DcMotor rightLift;
    public DcMotor leftLift;
    public DcMotorEx slideLift;
    public Servo leftServo;
    public static double ARM_POWER= 0.7;


    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideLift = hardwareMap.get(DcMotorEx.class, "slideLift");
        slideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftServo = hardwareMap.get(Servo.class, "leftServo");

        rightLift.setTargetPosition(0);
        leftLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int targetPosition = 0;
        double powerLeft, powerRight = 0;
        PIDController leftPID = new PIDController(0,0,0);
        PIDController rightPID = new PIDController(0,0,0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY*/)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Pose2d poseEstimate = drive.getPoseEstimate();

            if(gamepad1.right_bumper){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y*0.1,
                                -gamepad1.left_stick_x*0.1,
                                -gamepad1.right_stick_x*0.1
                        )
                );
            }
            else{
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y *0.5,
                                -gamepad1.left_stick_x*0.5,
                                -gamepad1.right_stick_x*0.5
                        )
                );
            }

            if(gamepad2.dpad_up)
            {
                if(gamepad2.left_bumper) {
                    rightLift.setTargetPosition(2150);
                    leftLift.setTargetPosition(2150);
                    leftLift.setPower(1);
                    rightLift.setPower(1);
                }else
                {
                    rightLift.setTargetPosition(2150);
                    leftLift.setTargetPosition(2150);
                    leftLift.setPower(ARM_POWER);
                    rightLift.setPower(ARM_POWER);
                }
            }
            else if(gamepad2.dpad_down)
            {
                rightLift.setTargetPosition(-2150);
                leftLift.setTargetPosition(-2150);
                leftLift.setPower(0.4);
                rightLift.setPower(0.4);
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


            drive.update();

            telemetry.addData("Encoder Right Lift-*", rightLift.getCurrentPosition());
            telemetry.addData("Encoder Left Lift", leftLift.getCurrentPosition());
            telemetry.addData("Max Speed = ", maxSpeed);
            telemetry.update();
        }

    }
}