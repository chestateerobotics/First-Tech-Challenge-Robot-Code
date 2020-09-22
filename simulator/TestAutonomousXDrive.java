package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "TestAutonomousXDrive", group = "XBot")
public class TestAutonomousXDrive extends OpMode {

  // Define harware
  private DcMotor m1, m2, m3, m4;
  private CRServo backServo;
  private DistanceSensor frontDistance, leftDistance, rightDistance, backDistance;
  private BNO055IMU imu;
  private BNO055IMU.Parameters parameters;
  private ColorSensor colorSensor;

  // Game variables
  private String alliance = "blue";
  private int target;
  private boolean isFinished = false;
  private int targetXMin, targetXMax, targetYMin, targetYMax;

    public void init() {
      m1 = hardwareMap.dcMotor.get("back_left_motor");
      m2 = hardwareMap.dcMotor.get("front_left_motor");
      m3 = hardwareMap.dcMotor.get("front_right_motor");
      m4 = hardwareMap.dcMotor.get("back_right_motor");
      m1.setDirection(DcMotor.Direction.REVERSE);
      m2.setDirection(DcMotor.Direction.REVERSE);
      m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      //GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
      imu = hardwareMap.get(BNO055IMU.class, "imu");

      backServo = hardwareMap.crservo.get("back_crservo");
      frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
      leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
      rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
      backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
      //gyro.init();

      parameters = new BNO055IMU.Parameters();
      parameters.accelerationIntegrationAlgorithm = null;
      parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
      parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
      parameters.calibrationData = null;
      parameters.calibrationDataFile = "";
      parameters.loggingEnabled = false;
      parameters.loggingTag = "Who cares.";

      imu.initialize(parameters);

      colorSensor = hardwareMap.colorSensor.get("color_sensor");
      telemetry.addData("Press Start When Ready", "");
      telemetry.update();

      target = (int) Math.ceil(Math.random() * 3);
      telemetry.addData("target", target);
      telemetry.update();
      if (target == 1) {
        targetXMin = 0;
        targetYMax = 0;
        targetXMax = 15;
        targetYMax = 15;
      } else if (target == 2) {
        targetXMin = 60;
        targetYMin = 60;
        targetXMax = 75;
        targetYMax = 75;
      } else {
        targetXMin = 0;
        targetYMin = 120;
        targetXMax = 15;
        targetYMax = 135;
      }
    }

    @Override
    public void loop() {
      if (!isFinished) {
        while (frontDistance.getDistance(DistanceUnit.CM) > targetYMax) {
          driveForward(1);
        }
        while (frontDistance.getDistance(DistanceUnit.CM) < targetYMin) {
          driveBackwards(1);
        }
        double returnX = leftDistance.getDistance(DistanceUnit.CM);
        while (leftDistance.getDistance(DistanceUnit.CM) > targetXMax) {
          driveLeft(1);
        }
        while (leftDistance.getDistance(DistanceUnit.CM) < targetXMin) {
          driveRight(1);
        }
        fullStop();
        telemetry.addData("Dropped the thing", "");
        telemetry.update();
        while (leftDistance.getDistance(DistanceUnit.CM) < returnX) {
          driveRight(1);
        }
        while (leftDistance.getDistance(DistanceUnit.CM) > returnX) {
          driveLeft(1);
        }
        while (!isOverWhite()) {
          driveBackwards(.3);
        }
        fullStop();
        isFinished = true;
        telemetry.addData("finished", "");
        telemetry.update();
      } else {
        stop();
      }
    }

    private void driveForward(double speed) {
      m1.setPower(speed);
      m2.setPower(speed);
      m3.setPower(speed);
      m4.setPower(speed);
    }

    private void driveBackwards(double speed) {
      m1.setPower(-speed);
      m2.setPower(-speed);
      m3.setPower(-speed);
      m4.setPower(-speed);
    }

    private void driveLeft(double speed) {
      m1.setPower(speed);
      m2.setPower(-speed);
      m3.setPower(speed);
      m4.setPower(-speed);
    }

    private void driveRight(double speed) {
      m1.setPower(-speed);
      m2.setPower(speed);
      m3.setPower(-speed);
      m4.setPower(speed);
    }

    private void rotateClockwise(double speed) {
      m1.setPower(speed);
      m2.setPower(speed);
      m3.setPower(-speed);
      m4.setPower(-speed);
    }

    private void rotateCounterClockwise(double speed) {
      m1.setPower(-speed);
      m2.setPower(-speed);
      m3.setPower(speed);
      m4.setPower(speed);
    }

    private void fullStop() {
      m1.setPower(0);
      m2.setPower(0);
      m3.setPower(0);
      m4.setPower(0);
    }

    private boolean isOverWhite() {
      return colorSensor.blue() > 200 &&
          colorSensor.green() > 200 &&
          colorSensor.red() > 200;
    }
}
