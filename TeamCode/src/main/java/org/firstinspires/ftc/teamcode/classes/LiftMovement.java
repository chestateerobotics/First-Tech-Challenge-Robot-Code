package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MOTOR_VELO_PIDS_LIFT;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kA_LIFT;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kStatic_LIFT;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV_LIFT;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleLift;

public class LiftMovement {
    private double targetPos = 0;
    private double targetVel = 0;
    private double targetAccel = 0;
    private double lastPos = 0;

    MotionProfile activeProfile = null;
    private PIDFController controller = null;
    SampleLift lift = null;

    double profileStart = 0;
    NanoClock clock = NanoClock.system();


    public LiftMovement(HardwareMap hardwareMap)
    {
        controller = new PIDFController(MOTOR_VELO_PIDS_LIFT, kV_LIFT, kA_LIFT, kStatic_LIFT);
        lift = new SampleLift(hardwareMap);
    }



    private MotionProfile profileGenerate()
    {
        MotionState start = new MotionState(lastPos, 0, 0, 0);
        MotionState goal = new MotionState(targetPos, 0, 0, 0);
        lastPos =  targetPos;
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, targetVel, targetAccel);
    }

    public void startMovement(double targetPos, double targetVel, double targetAccel)
    {
        this.targetPos = targetPos;
        this.targetVel = targetVel;
        this.targetAccel = targetAccel;
        activeProfile = profileGenerate();
        profileStart = clock.seconds();
        controller.reset();
    }

    public double powerLift()
    {
        double profileTime = clock.seconds() - profileStart;
        if (profileTime > activeProfile.duration()) {
            return 0;
        }

        MotionState motionState = activeProfile.get(profileTime);
        controller.setTargetPosition(motionState.getX());
        controller.setTargetVelocity(motionState.getV());
        controller.setTargetAcceleration(motionState.getA());
        double vel = lift.getWheelVelocities().get(0);
        double pos = lift.getWheelPositions().get(0);
        return controller.update(pos, vel);
    }

}
