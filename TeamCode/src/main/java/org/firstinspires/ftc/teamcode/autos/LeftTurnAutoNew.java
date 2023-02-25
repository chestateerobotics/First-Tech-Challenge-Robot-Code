package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.classes.LiftMovement;
import org.firstinspires.ftc.teamcode.classes.WebcamClass;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleLift;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;
import java.util.Objects;

@Config
@Autonomous
public class LeftTurnAutoNew extends LinearOpMode
{

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private VoltageSensor ControlHub_VoltageSensor;
    private SampleLift lift = new SampleLift(hardwareMap);
    private LiftMovement power = new LiftMovement(hardwareMap);
    public int encoderSubtracter = 0;
    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");



        waitForStart();
        while (opModeIsActive()) {





            packet.put("Right Power", power.getPower());
            packet.put("Left Power", power.getPower());
            packet.put("Right Velocity", lift.getWheelVelocities().get(0));
            packet.put("Left Velocity", lift.getWheelVelocities().get(1));
            packet.put("Right Position", lift.getWheelPositions().get(0));
            packet.put("Left Position", lift.getWheelPositions().get(1));
            packet.put("Voltage", ControlHub_VoltageSensor.getVoltage());
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

            break;
        }

    }
    private void encoderArm(int level, boolean toggle){
        if(toggle) {
            if (level == 0) {
                power.startMovement(0, 20, 20);
                lift.setMotorPowers(power.powerLift(), power.powerLift(), 0, 0);
            } else if (level == 1) {
                power.startMovement(17.36, 20, 20);
                lift.setMotorPowers(power.powerLift(), power.powerLift(), 0,0);
            } else if (level == 2) {
                power.startMovement(27.36, 20, 20);
                lift.setMotorPowers(power.powerLift(), power.powerLift(),0,0);
            } else if (level == 3) {
                power.startMovement(37.36, 20, 20);
                lift.setMotorPowers(power.powerLift(), power.powerLift(),0,0);
            }
            else if(level == 5){
                //rightLift.setTargetPosition(rightLift.getCurrentPosition()-500);
                //leftLift.setTargetPosition(rightLift.getCurrentPosition()-500);
                //rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                power.startMovement(0, 20, 20);
                lift.setMotorPowers(power.powerLift(), power.powerLift(),0,0);
            }
            else {
                //rightLift.setTargetPosition(300-encoderSubtracter);
                //leftLift.setTargetPosition(300-encoderSubtracter);
                //rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                power.startMovement(0, 20, 20);
                lift.setMotorPowers(power.powerLift(), power.powerLift(),0,0);
            }

        }
        lift.setMotorPowers(0, 0,0,0);
    }
}