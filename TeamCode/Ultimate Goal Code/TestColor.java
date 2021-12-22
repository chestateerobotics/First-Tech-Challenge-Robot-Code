package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp

public class TestColor extends OpMode{
    private ColorSensor colorSensor;
    private AnalogInput pot;
    
    @Override
    public void init(){
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        pot = hardwareMap.get(AnalogInput.class, "pot");
        
    }
    
    @Override
    public void loop(){
        telemetry.addData("Color Sensor Green", colorSensor.green());
        telemetry.addData("Color Sensor Red", colorSensor.red());
        telemetry.addData("Color Sensor Blue", colorSensor.blue());
        telemetry.addData("Color Sensor Hue", colorSensor.argb());
        telemetry.addData("Color Sensor amount of light", colorSensor.alpha());
        colorSensor.enableLed(false);
        
        telemetry.addData("Potentimator voltage is", pot.getVoltage());
    }
}