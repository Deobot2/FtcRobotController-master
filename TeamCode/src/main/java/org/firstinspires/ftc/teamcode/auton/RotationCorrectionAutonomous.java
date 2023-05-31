/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RotationCorrectionAutonomous extends LinearOpMode
{
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_left;
    private BNO055IMU imu;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    boolean armLifted = true;
    boolean strafeDone = false;
    int currentStep = 0;
    float desiredAngle;
    int strafeMode = 1;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 13;
    int MIDDLE = 14;
    int RIGHT = 15;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        BNO055IMU.Parameters imuParameters;
        Orientation angles;

        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            front_right.setDirection(DcMotorSimple.Direction.REVERSE);
            back_left.setDirection(DcMotorSimple.Direction.REVERSE);

            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Current Heading", angles.firstAngle);
            telemetry.addData("Back Left Motor", back_left.getCurrentPosition());
            telemetry.addData("Back Left Power", back_left.getPower());
            telemetry.addData("Strafe Mode", strafeMode);
            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            /* Actually do something useful */
            if (currentStep == 0){
                desiredAngle = angles.firstAngle;
                currentStep = 1;
            }
            telemetry.addData("Desired Heading", desiredAngle);
            if (back_left.getCurrentPosition() > -2500 && currentStep == 1){
                //strafeLeftTest(-0.5);
                strafeLeft(-0.5, angles.firstAngle, desiredAngle);
            }
            else{
                if (currentStep == 1){
                    zeroPower();
                    currentStep = 2;
                }
            }
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.addData("Back Left Encoder", back_left.getCurrentPosition());
        telemetry.addData("Tag", tagOfInterest.id);
    }
    private void zeroPower(){
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);
    }
    private void moveForward(double x){
        back_left.setPower(x);
        back_right.setPower(x);
        front_left.setPower(x);
        front_right.setPower(x);
    }
    private void moveBackward(double x){
        back_left.setPower(-x);
        back_right.setPower(-x);
        front_left.setPower(-x);
        front_right.setPower(-x);
    }
    private void strafeLeftTest(double x){
        back_left.setPower(x);
        back_right.setPower(-x);
        front_left.setPower(-x);
        front_right.setPower(x);
    }
    private void strafeLeft(double x, float heading, float desiredHeading){
        if ((heading < desiredHeading + 5 && heading > desiredHeading - 5) || (heading > desiredHeading + 5 && heading < desiredHeading - 5)){
            back_left.setPower(x);
            back_right.setPower(-x);
            front_left.setPower(-x);
            front_right.setPower(x);
            strafeMode = 1;
        }
        else if ((heading < desiredHeading && desiredHeading < 0) || (heading > desiredHeading && desiredHeading > 0)){
            back_left.setPower(x + x/4);
            back_right.setPower(-x - x/4);
            front_left.setPower(-x + x/4);
            front_right.setPower(x - x/4);
            strafeMode = 2;
        }
        else if ((heading > desiredHeading && desiredHeading < 0) || (heading < desiredHeading && desiredHeading > 0)) {
            back_left.setPower(x - x/4);
            back_right.setPower(-x + x/4);
            front_left.setPower(-x - x/4);
            front_right.setPower(x + x/4);
            strafeMode = 3;
        }
    }
    private void strafeRight(double x){
        back_left.setPower(-x);
        back_right.setPower(x);
        front_left.setPower(x);
        front_right.setPower(-x);
    }
    private void rotateLeft(double x){
        back_left.setPower(-x);
        back_right.setPower(x);
        front_left.setPower(-x);
        front_right.setPower(x);
    }
    private void rotateRight(double x){
        back_left.setPower(x);
        back_right.setPower(-x);
        front_left.setPower(x);
        front_right.setPower(-  x);
    }
}