package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class RedCarouselSide extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private boolean goWarehouse = false;
    private double maxSpeed = 0.4;

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

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Going to Warehouse", goWarehouse);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Go to carousel, turn 45 degrees, and turn motor
            move45Left(450);
            doTurn(45);
            moveForward(-200);
            //Turn carousel motor
            //carousel.setPower(1);
            sleep(5000);
            moveForward(200);
            doTurn(-45);
            move45Left(-400);
            moveSide(750);
            moveForward(460);
            sleep(6000);
            moveForward(-460);
            moveSide(-750);
            move45Left(940);

            telemetry.addData("Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", frontRight.getCurrentPosition());
            telemetry.addData("Back Left", backLeft.getCurrentPosition());
            telemetry.addData("Back Right", backRight.getCurrentPosition());
            telemetry.update();
            break;
        }
    }
    private void setSpeeds(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
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

        frontLeft.setPower(maxSpeed);
        frontRight.setPower(maxSpeed);
        backLeft.setPower(maxSpeed);
        backRight.setPower(maxSpeed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(300);
    }

    private void moveSide(double distance){
        int num = (int)calculateEncoders(distance);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + num);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - num);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - num);
        backRight.setTargetPosition(backRight.getCurrentPosition() + num);

        frontLeft.setPower(maxSpeed);
        frontRight.setPower(maxSpeed);
        backLeft.setPower(maxSpeed);
        backRight.setPower(maxSpeed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(300);
    }
    private void doTurn(int degree){
        int num = (int)((degree / 360.0) * 4350);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + num);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - num);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + num);
        backRight.setTargetPosition(backRight.getCurrentPosition() - num);

        frontLeft.setPower(maxSpeed);
        frontRight.setPower(maxSpeed);
        backLeft.setPower(maxSpeed);
        backRight.setPower(maxSpeed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(300);
    }

    private void move45Right(double distance){
        int num = (int)calculateEncoders(distance * (4.0/3.0));
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + num);
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        backLeft.setTargetPosition(backLeft.getCurrentPosition());
        backRight.setTargetPosition(backRight.getCurrentPosition() + num);

        frontLeft.setPower(maxSpeed);
        frontRight.setPower(maxSpeed);
        backLeft.setPower(maxSpeed);
        backRight.setPower(maxSpeed);

        while(frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(300);
    }

    private void move45Left(double distance){
        int num = (int)calculateEncoders(distance * (4/3.0));
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + num);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + num);
        backRight.setTargetPosition(backRight.getCurrentPosition());

        frontLeft.setPower(maxSpeed);
        frontRight.setPower(maxSpeed);
        backLeft.setPower(maxSpeed);
        backRight.setPower(maxSpeed);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()){
            telemetry.addData("Encoder Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(300);
    }
}