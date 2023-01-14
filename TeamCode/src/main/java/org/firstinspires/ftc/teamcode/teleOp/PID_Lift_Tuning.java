package org.firstinspires.ftc.teamcode.teleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "PID Test")
public class PID_Lift_Tuning extends LinearOpMode {

    DcMotorEx rightLift, leftLift;

    ElapsedTime leftTime = new ElapsedTime();
    ElapsedTime rightTime = new ElapsedTime();

    private double leftLastError = 0;
    private double rightLastError = 0;
    private double integralSumLeft, integralSumRight = 0;

    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double targetPosition = 2000;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive())
        {
            double powerRight = returnRightPower(targetPosition, rightLift.getCurrentPosition());
            double powerLeft = returnLeftPower(targetPosition, leftLift.getCurrentPosition());
            packet.put("Right Power", powerRight);
            packet.put("Left Power", powerLeft);
            packet.put("Right Position", rightLift.getCurrentPosition());
            packet.put("Left Position", leftLift.getCurrentPosition());
            packet.put("Right Error", rightLastError);
            packet.put("Left Error", leftLastError);


            rightLift.setPower(powerRight);
            leftLift.setPower(powerLeft);

            dashboard.sendTelemetryPacket(packet);

            sleep(500);
            powerRight = returnRightPower(-targetPosition, rightLift.getCurrentPosition());
            powerLeft = returnLeftPower(-targetPosition, leftLift.getCurrentPosition());
            packet.put("Right Power", powerRight);
            packet.put("Left Power", powerLeft);
            packet.put("Right Position", rightLift.getCurrentPosition());
            packet.put("Left Position", leftLift.getCurrentPosition());
            packet.put("Right Error", rightLastError);
            packet.put("Left Error", leftLastError);

            rightLift.setPower(powerRight);
            leftLift.setPower(powerLeft);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    public double returnRightPower(double reference, double state)
    {
        double error = reference - state;

        // forward euler integration
        integralSumRight += error * rightTime.seconds();
        double derivative = (error - rightLastError) / rightTime.seconds();

        double output = (error * Kp) + (integralSumRight * Ki) + (derivative * Kd);

        rightTime.reset();
        rightLastError = error;

        return output;
    }

    public double returnLeftPower(double reference, double state)
    {
        double error = reference - state;

        // forward euler integration
        integralSumLeft += error * leftTime.seconds();
        double derivative = (error - leftLastError) / leftTime.seconds();

        double output = (error * Kp) + (integralSumLeft * Ki) + (derivative * Kd);

        leftTime.reset();
        leftLastError = error;

        return output;
    }
}
