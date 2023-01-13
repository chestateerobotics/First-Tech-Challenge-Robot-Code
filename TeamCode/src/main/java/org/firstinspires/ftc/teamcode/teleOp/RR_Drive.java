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
    public DcMotorEx rightLift;
    public DcMotorEx leftLift;
    public Servo leftServo;

    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftServo = hardwareMap.get(Servo.class, "leftServo");

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
            Pose2d poseEstimate = drive.getPoseEstimate();

            if(gamepad1.right_bumper){
                drive.OMEGA_WEIGHT = 0.2;
                drive.VX_WEIGHT = 0.2;
                drive.VY_WEIGHT= 0.2;
            }
            else{
                drive.OMEGA_WEIGHT = 0.7;
                drive.VX_WEIGHT = 0.7;
                drive.VY_WEIGHT= 0.7;
            }

            if(gamepad2.dpad_up)
            {
                targetPosition = 2150;
                powerLeft = leftPID.output(targetPosition, leftLift.getCurrentPosition());
                powerRight = rightPID.output(targetPosition, leftLift.getCurrentPosition());
                leftLift.setPower(powerLeft);
                rightLift.setPower(powerRight);

            }
            else if(gamepad2.dpad_down)
            {
                targetPosition = -2150;
                powerLeft = leftPID.output(targetPosition, leftLift.getCurrentPosition());
                powerRight = rightPID.output(targetPosition, leftLift.getCurrentPosition());
                leftLift.setPower(powerLeft);
                rightLift.setPower(powerRight);
            }
            else{
                targetPosition = (rightLift.getCurrentPosition() + leftLift.getCurrentPosition())/2;
                powerLeft = leftPID.output(targetPosition, leftLift.getCurrentPosition());
                powerRight = rightPID.output(targetPosition, leftLift.getCurrentPosition());
                leftLift.setPower(powerLeft);
                rightLift.setPower(powerRight);
            }

            if(gamepad2.a)
            {
                leftServo.setPosition(-1);
            }
            else
            {
                leftServo.setPosition(1);
            }

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            telemetry.addData("Encoder Right Lift-*", rightLift.getCurrentPosition());
            telemetry.addData("Encoder Left Lift", leftLift.getCurrentPosition());
            telemetry.addData("Max Speed = ", maxSpeed);
            telemetry.update();
        }

    }
}
