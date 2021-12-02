package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class EncoderTests extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
            if(gamepad1.dpad_up){
                frontLeft.setPower(0.5);
            }
            else if(gamepad1.dpad_right){
                frontRight.setPower(0.5);
            }
            else if(gamepad1.dpad_down){
                backLeft.setPower(0.5);
            }
            else if(gamepad1.dpad_left){
                backRight.setPower(0.5);
            }
            else{
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }


            moveForward(660.4);
            sleep(2000);
            moveSide(660.4);
            sleep(2000);
            moveAngle(500, 45);
            sleep(2000);
            */
            //telemetry.addData("Arc Length ", calculateArcLength(180));
            //telemetry.update();
            //sleep(1000);
            moveForward(660.4);
            sleep(200);
            doTurn(90);
            sleep(200);
            moveForward(660.4);
            sleep(500);
            telemetry.addData("Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", frontRight.getCurrentPosition());
            telemetry.addData("Back Left", backLeft.getCurrentPosition());
            telemetry.addData("Back Right", backRight.getCurrentPosition());
            telemetry.update();
            break;
        }
    }
    private void setSpeeds(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        /*
        double largest;
        largest = Math.max(largest, Math.abs(flSpeed));
        largest = Math.max(largest, Math.abs(frSpeed));
        largest = Math.max(largest, Math.abs(blSpeed));
        largest = Math.max(largest, Math.abs(brSpeed));
        */

        frontLeft.setPower(flSpeed);
        frontRight.setPower(frSpeed);
        backLeft.setPower(blSpeed);
        backRight.setPower(brSpeed);
    }

    private void driveMecanum(double forward, double strafe, double rotate) {
        double frontLeftSpeed = forward + strafe + rotate;
        double frontRightSpeed = forward - strafe - rotate;
        double backLeftSpeed = forward - strafe + rotate;
        double backRightSpeed = forward + strafe - rotate;

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    private double calculateEncoders(double distance){
        int diameter = 96; //In mm
        double encoders = 537.7;
        double revolutions = distance / (diameter * Math.PI);
        return revolutions * encoders;
    }

    private double calculateArcLength(int degree){
        double wheelDiameter = 96; //In mm
        double distanceDiameter = 419.1; //In mm (533.4)
        double encoders = 537.7;
        double arcLength = (degree/360.0) * (distanceDiameter * Math.PI);

        double revs = arcLength / (wheelDiameter * Math.PI);

        return revs * encoders;
    }

    private void moveForward(double distance){
        int num = (int)calculateEncoders(distance);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + num);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + num);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + num);
        backRight.setTargetPosition(backRight.getCurrentPosition() + num);

        frontLeft.setPower(0.2);
        frontRight.setPower(0.2);
        backLeft.setPower(0.2);
        backRight.setPower(0.2);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(100);
    }

    private void moveSide(double distance){
        int num = (int)calculateEncoders(distance);

        frontLeft.setTargetPosition(-frontLeft.getCurrentPosition() + num);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + num);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + num);
        backRight.setTargetPosition(-backRight.getCurrentPosition() + num);

        frontLeft.setPower(0.2);
        frontRight.setPower(0.2);
        backLeft.setPower(0.2);
        backRight.setPower(0.2);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(100);
    }
    /*
        private void doTurn(int degree){
            int arc = (int)calculateArcLength(degree);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - arc);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + arc);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() - arc);
            backRight.setTargetPosition(backRight.getCurrentPosition() + arc);

            frontLeft.setPower(0.2);
            frontRight.setPower(0.2);
            backLeft.setPower(0.2);
            backRight.setPower(0.2);

            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
                telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
                telemetry.addData("Front Left Power", frontLeft.getPower());
                telemetry.update();
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            sleep(100);
        }
    */
    private void doTurn(int degree){
        int num = (int)((degree / 360.0) * 4350);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + num);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - num);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + num);
        backRight.setTargetPosition(backRight.getCurrentPosition() - num);

        frontLeft.setPower(0.3);
        frontRight.setPower(0.3);
        backLeft.setPower(0.3);
        backRight.setPower(0.3);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(100);
    }

    private void move45Right(double distance){
        int num = (int)calculateEncoders(distance);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + num);
        frontRight.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(backRight.getCurrentPosition() + num);



        frontLeft.setPower(0.2);
        frontRight.setPower(0.2);
        backLeft.setPower(0.2);
        backRight.setPower(0.2);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(100);
    }
    private void move45Left(double distance){
        int num = (int)calculateEncoders(distance);
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + num);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + num);
        backRight.setTargetPosition(0);


        frontLeft.setPower(0.2);
        frontRight.setPower(0.2);
        backLeft.setPower(0.2);
        backRight.setPower(0.2);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(100);
    }
}
