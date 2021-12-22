package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//Program is just the driving OpMode for the robot

@TeleOp(name = "Only Driving OPMode")
public class MechanumOpMode extends OpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double[] distances;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);

    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;


        mecanumDrive.driveMecanum(forward, strafe, rotate);
        
        telemetry.addData("Front Left Power = ", mecanumDrive.frontLeft.getPower());
            telemetry.addData("Front Right Power = ", mecanumDrive.frontRight.getPower());
            telemetry.addData("Back Left Power = ", mecanumDrive.backLeft.getPower());
            telemetry.addData("Back Right Power = ", mecanumDrive.backRight.getPower());
            telemetry.update();
    }
}