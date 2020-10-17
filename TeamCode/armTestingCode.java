package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "WobbleGoalArm")
public class WobbleGoalArm extends LinearOpMode{
    public void runOpMode() {
        waitForStart();
        double maxSpeed = 0.8;
        DcMotor arm = hardwareMap.dcMotor.get("arm_motor");
        Servo armServo = hardwareMap.servo.get("hand_servo");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (opModeIsActive()) {
            if (gamepad1.a){
                maxSpeed = 1.0;
            }
            else if (gamepad1.b){
                maxSpeed = 0.8;
            }
            else if (gamepad1.x){
                double position = armServo.getPosition();
                if (position == 1.0){
                    armServo.setPosition(0);
                }
                else {
                    armServo.setPosition(1.0);
                }
            }

            if (gamepad1.left_bumper){
                arm.setPower(maxSpeed);
            }
            else if (gamepad1.right_bumper){
                arm.setPower(-maxSpeed);
            }
            else{
                arm.setPower(0);
            }
            telemetry.addData("Max speed", "%.1f", maxSpeed);
            telemetry.addData("Arm Power", "%.1f", arm.getPower());
            telemetry.addData("Servo Position","%.1f",armServo.getPosition());
        }
    }
}