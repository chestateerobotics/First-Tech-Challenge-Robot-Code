package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp
public class ConceptVuforiaWebcam extends LinearOpMode
{
    final double MM_PER_INCH = 25.40 ;   //  Metric conversion
    private static final String VUFORIA_KEY =
            "AcUxfhT/////AAABmdkSEZN8P0ZHok0KoUe9PvFF3PTxjUK/PifUJGID6XD+ymIfA+JZj8JlKWr5Ty4Z/kd6TmB1bpbgZ7rQx+eyQPPWyP9Z/XDjLLTXEF/t/T0CyUCqgrXAYY9E1oIVNLesakTVsxv7RafPgJENodWFwBjxLelCw80jD8I5VjzlJscxN4IH7A/a3fZULcNCUtrSYLXZX0dDgJ1CtNi7j8ZLEvYgzaB3qjZhc3H/9paRKhd7+D52EgDC/Jz7RZ8T8/I01MInCOtea7Owh6DvRgYJBj8+Km9mrMwil20Q+oTsbTCe41l/k+atc45z3nIQ2u4RMUE0XyCYWtWXj4EXSZkl0tDdGULsXfQYJT5p7eMPIni9";

    VuforiaLocalizer vuforia    = null;
    OpenGLMatrix targetPose     = null;
    String targetName           = "";

    public void runOpMode(){
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * To get an on-phone camera preview, use the code below.
         * If no camera preview is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the trackable objects from the Assets file, and give them meaningful names
        VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset("RedSoloCup_OT");
        targetsFreightFrenzy.get(0).setName("RedSoloCupTarget");

        // Start tracking targets in the background
        targetsFreightFrenzy.activate();

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        boolean targetFound     = false;    // Set to true when a target is detected by Vuforia
        double  targetRange     = 0;        // Distance from camera to target in Inches
        double  targetBearing   = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.

        while (opModeIsActive()){
            // Look for first visible target, and save its pose.
            targetFound = false;
            for (VuforiaTrackable trackable : targetsFreightFrenzy){
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()){
                    targetPose = ((VuforiaTrackableDefaultListener)trackable.getListener()).getVuforiaCameraFromTarget();

                    // if we have a target, process the "pose" to determine the position of the target relative to the robot.
                    if (targetPose != null){
                        targetFound = true;
                        targetName  = trackable.getName();
                        VectorF trans = targetPose.getTranslation();

                        // Extract the X & Y components of the offset of the target relative to the robot
                        double targetX = trans.get(0) / MM_PER_INCH; // Image X axis
                        double targetY = trans.get(2) / MM_PER_INCH; // Image Z axis

                        // target range is based on distance from robot position to origin (right triangle).
                        targetRange = Math.hypot(targetX, targetY);

                        // target bearing is based on angle formed between the X axis and the target range line
                        targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));

                        break;  // jump out of target tracking loop if we find a target.
                    }
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", " %s", targetName);
                telemetry.addData("Range",  "%5.1f inches", targetRange);
                telemetry.addData("Bearing","%3.0f degrees", targetBearing);
            } else {
                telemetry.addData(">","Drive using joystick to find target\n");
            }

            // Drive to target Automatically if Left Bumper is being pressed, AND we have found a target.
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading and range error so we can use them to control the robot automatically.
                double rangeError = targetRange;
                double headingError = targetBearing;
            }

            telemetry.update();
            sleep(10);
        }
    }
}
