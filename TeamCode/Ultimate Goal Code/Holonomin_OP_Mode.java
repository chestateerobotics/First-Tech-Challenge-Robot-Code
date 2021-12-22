package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;


@TeleOp(name = "Main Drive")
public class Holonomin_OP_Mode extends LinearOpMode {
    public void runOpMode() {

        double maxSpeed = 1;
        DcMotor front = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor left = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor right = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor back = hardwareMap.dcMotor.get("motorBackLeft");
        front.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.REVERSE);
        front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double lsY = gamepad1.left_stick_y;
            if (Math.abs(lsY) <= 0.1) lsY = 0;
            double lsX = gamepad1.left_stick_x;
            if (Math.abs(lsX) <= 0.1) lsX = 0;
            double rsX = gamepad1.right_stick_x;
            if (Math.abs(rsX) <=0.1) rsX = 0;
            if (gamepad1.left_bumper) maxSpeed = 0.5;
            if (gamepad1.right_bumper) maxSpeed = 1;

            front.setPower(lsX+rsX*maxSpeed);
            back.setPower(lsX+rsX*maxSpeed);
            left.setPower(lsY+rsX*maxSpeed);
            right.setPower(lsY+rsX*maxSpeed);

            telemetry.addData("Max speed", "%.1f", maxSpeed);
            telemetry.addData("Left motor", "%.1f", left.getPower());
            telemetry.addData("Right motor", "%.1f", right.getPower());
            telemetry.addData("Up motor", "%.1f", front.getPower());
            telemetry.addData("Down motor", "%.1f", back.getPower());
            telemetry.addData("Left Stick X", "%.1f", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", "%.1f", -gamepad1.left_stick_y);
            telemetry.addData("Right Stick X", "%.1f", gamepad1.right_stick_x);
            telemetry.addData("Encoders"," %d %d %d %d", front.getCurrentPosition(), right.getCurrentPosition(),
                    left.getCurrentPosition(), back.getCurrentPosition());
            telemetry.update();
        }
        front.setPower(0);
        back.setPower(0);
        left.setPower(0);
        right.setPower(0);
    }
}


