package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class RedBlandAutonomous extends LinearOpMode{
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private DcMotor arm_right;
    private DcMotor arm_left;
    public void runOpMode() {
        mecanumDrive.init(hardwareMap);
        arm_right = hardwareMap.get(DcMotor.class, "arm_right");
        arm_left = hardwareMap.get(DcMotor.class, "arm_left");
        
        arm_right.setDirection(DcMotor.Direction.REVERSE);
        
        arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            moveArm(0.3);
            sleep(1000);
            moveArm(0.1);
            
            mecanumDrive.driveMecanum(-1, 0, 0);
            sleep(500);
            mecanumDrive.driveMecanum(0, 0, 0);
            break;
        }
    }
    private void moveArm(double speed){
        arm_left.setPower(speed);
        arm_right.setPower(speed);
    }
}