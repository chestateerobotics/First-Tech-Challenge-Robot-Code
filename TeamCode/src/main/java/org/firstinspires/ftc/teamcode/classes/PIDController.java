package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDController {

    double Kp;
    double Ki;
    double Kd;
    double lastError = 0;
    double integral = 0;

    ElapsedTime timer = new ElapsedTime();

    /**
     * Set PID gains
     * @param Kp proportional gain
     * @param Ki integral gain
     * @param Kd derivative gain
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /**
     * calculate PID output given the reference and the current system state
     * @param reference where we would like our system to be
     * @param state where our system is
     * @return the signal to send to our motor or other actuator
     */
    public double output(double reference, double state) {
        double error = reference - state;

        // forward euler integration
        integral += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        double output = (error * Kp) + (integral * Ki) + (derivative * Kd);

        timer.reset();
        lastError = error;

        return output;
    }



}