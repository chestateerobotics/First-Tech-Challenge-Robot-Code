package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class Movement extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double maxSpeed = 1;

    public void runOpMode() {
        mecanumDrive.init(hardwareMap);

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

            mecanumDrive.driveMecanum(forward, strafe, rotate);

            telemetry.addData("Max Speed = ", maxSpeed);
            telemetry.update();
        }
    }
}
