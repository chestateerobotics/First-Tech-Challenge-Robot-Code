package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp

public class MotorSpin extends OpMode{
    private DcMotor motor1;
    private DcMotor motor2;
    
    @Override
    public void init(){
        motor1 = hardwareMap.dcMotor.get("LauncherFrontLeft");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2 = hardwareMap.dcMotor.get("LauncherFrontRight");
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    }
    @Override
    public void loop(){
        motor1.setPower(1);
        motor2.setPower(1);
        
    }
}