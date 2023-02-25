package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple lift motor hardware implementation for REV hardware.
 */
@Config
public class SampleLift extends MecanumDrive {
    public static double LATERAL_MULTIPLIER = 1;
    public DcMotorEx leftLift, rightLift;
    private List<DcMotorEx> liftMotors;

    private VoltageSensor batteryVoltageSensor;

    public SampleLift(HardwareMap hardwareMap) {
        super(DriveConstants.kV_LIFT, DriveConstants.kA_LIFT, DriveConstants.kStatic_LIFT, DriveConstants.TRACK_WIDTH, DriveConstants.TRACK_WIDTH, LATERAL_MULTIPLIER);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        liftMotors = Arrays.asList(leftLift, rightLift);

        for (DcMotorEx motor : liftMotors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (DriveConstants.RUN_USING_ENCODER_LIFT) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (DriveConstants.RUN_USING_ENCODER_LIFT && DriveConstants.MOTOR_VELO_PIDS_LIFT != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PIDS_LIFT);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

    }


    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : liftMotors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : liftMotors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.kP, coefficients.kI, coefficients.kD, 0);

        for (DcMotorEx motor : liftMotors) {
            //motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : liftMotors) {
            wheelPositions.add(DriveConstants.encoderTicksToInchesLift(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : liftMotors) {
            wheelVelocities.add(DriveConstants.encoderTicksToInchesLift(motor.getVelocity()));
        }
        return wheelVelocities;
    }





    public void moveLift()
    {

    }

    @Override
    protected double getRawExternalHeading() {
        return 0;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftLift.setPower(v);
        rightLift.setPower(v1);
    }
}
