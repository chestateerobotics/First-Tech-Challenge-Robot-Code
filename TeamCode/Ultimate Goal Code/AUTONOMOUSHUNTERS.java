package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous


public class AUTONOMOUSHUNTERS extends OpMode {


    // Define hardware
    private DcMotor m1, m2, m3, m4;
    private ColorSensor colorSensor;

    // Game variables
    private String alliance = "blue";
    private boolean isFinished = false;

    public void init() {
        m1 = hardwareMap.dcMotor.get("motorFrontRight");
        m2 = hardwareMap.dcMotor.get("motorFrontLeft");
        m3 = hardwareMap.dcMotor.get("motorBackRight");
        m4 = hardwareMap.dcMotor.get("motorBackLeft");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
     

        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Press Start When Ready", "");
        telemetry.update();

    }

    @Override
    public void loop() {
        if (!isFinished) {
            while (!isOverWhite()) {
                telemetry.addData("R: ", colorSensor.red());
                telemetry.addData("G: ", colorSensor.green());
                telemetry.addData("B: ", colorSensor.blue());
                telemetry.update();
                driveForward(.5);
            }
            fullStop();
            isFinished = true;
            telemetry.addData("finished", "");
            telemetry.update();
        }
        else {
            stop();
        }
    }

    private void driveForward(double speed) {
        
    }

    private void fullStop() {
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

    private boolean isOverWhite() {
        return colorSensor.blue() > 150 &&
                colorSensor.green() > 150 &&
                colorSensor.red() > 150;
    }
}