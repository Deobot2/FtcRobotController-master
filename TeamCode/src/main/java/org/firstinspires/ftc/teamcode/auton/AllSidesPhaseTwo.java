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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
public class AllSidesPhaseTwo extends LinearOpMode
{
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotorEx back_right;
    private DcMotor front_left;
    private DcMotor armExtend;
    private Servo claw;
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
    double pwrLevel = 0.38;
    int tempEncoder = 0;
    int tempEncoderAgain = 0;
    int currentStep = 0;
    int mediumJunctionLimit = 1600; // Temporary. Need to change value
    int startHeight = 300;

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
        back_right = hardwareMap.get(DcMotorEx.class, "back_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        claw = hardwareMap.get(Servo.class, "claw");
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
//          DO NOT UNCOMMENT THESE. They break everything and I have no idea why
//            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            zeroPower();

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

            telemetry.addData("Step: ", currentStep);
            telemetry.addData("Back Left: ", back_left.getCurrentPosition());
            telemetry.addData("Back Left Power", back_left.getPower());
            telemetry.addData("Back Right Power", back_right.getPower());

            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            /* Lift arm before actually doing something useful */
            if (armExtend.getCurrentPosition() < startHeight && currentStep == 0){
                extendArm();
            }
            else{
                if (currentStep == 0){
                    zeroPower();
                    currentStep = 1;
                }
            }

            /* Actually do something useful */
            if (back_left.getCurrentPosition() > -65 && currentStep == 1){
                moveForward(0.35);
            }
            else{
                if (currentStep == 1) {
                    currentStep = 2;
                }
            }
            if (back_left.getCurrentPosition() > -1345 && currentStep == 2){
                strafeLeft(0.35);
            }
            else{
                if (currentStep == 2) {
                    currentStep = 3;
                }
            }
            if (back_left.getCurrentPosition() > -2466 && currentStep == 3){
                moveForward(0.9);
            }
            else{
                if (currentStep == 3) {
                    currentStep = 4;
                }
            }
            if (angles.firstAngle > -45 && currentStep == 4){
                rotateRight(0.2);
            }
            else{
                if (currentStep == 4) {
                    zeroPower();
                    currentStep = 5;
                    tempEncoder = back_left.getCurrentPosition();
                }
            }
            if (armExtend.getCurrentPosition() < mediumJunctionLimit && currentStep == 5){
                extendArm();
            }
            else{
                if (currentStep == 5){
                    zeroPower();
                    currentStep = 6;
                }
            }
            if (tempEncoder - back_left.getCurrentPosition() < 75 && currentStep == 6){
                moveForward(0.2);
            }
            else{
                if (currentStep == 6) {
                    zeroPower();
                    currentStep = 7;
                }
            }
            if (claw.getPosition() != 1 && currentStep == 7){
                claw.setPosition(1);
            }
            else{
                if (currentStep == 7){
                    currentStep = 8;
                }
            }
            if (tempEncoder - back_left.getCurrentPosition() > 0 && currentStep == 8){
                moveBackward(0.2);
            }
            else{
                if (currentStep == 8) {
                    zeroPower();
                    currentStep = 9;
                }
            }
            if (armExtend.getCurrentPosition() > 0 && currentStep == 9){
                retractArm();
            }
            else{
                if (currentStep == 9){
                    zeroPower();
                    currentStep = 10;
                }
            }
            if (angles.firstAngle < 0 && currentStep == 10){
                rotateLeft(0.2);
            }
            else{
                if (currentStep == 10) {
                    zeroPower();
                    tempEncoderAgain = back_left.getCurrentPosition();
                    currentStep = 11;
                }
            }
            if (currentStep == 11){
                if (tagOfInterest.id == MIDDLE){
                    if (back_left.getCurrentPosition() < tempEncoderAgain + 1280){
                        strafeRight(0.65);
                    }
                    else{
                        zeroPower();
                    }
                }
                else if (tagOfInterest.id == RIGHT){
                    if (back_left.getCurrentPosition() < tempEncoderAgain + 2560){
                        strafeRight(0.65);
                    }
                    else{
                        zeroPower();
                    }
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
    private void  extendArm(){
        armExtend.setPower(0.5);
    }
    private void retractArm(){
        armExtend.setPower(-0.35);
    }
    private void zeroPower(){
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);
        armExtend.setPower(0);
    }
    private void moveForward(double x){
        back_left.setPower(-x);
        back_right.setPower(-x);
        front_left.setPower(-x);
        front_right.setPower(-x);
    }
    private void moveBackward(double x){
        back_left.setPower(x);
        back_right.setPower(x);
        front_left.setPower(x);
        front_right.setPower(x);
    }
    private void strafeLeft(double x){
        back_left.setPower(-x);
        back_right.setPower(x);
        front_left.setPower(x);
        front_right.setPower(-x);
    }
    private void strafeRight(double x){
        back_left.setPower(x);
        back_right.setPower(-x);
        front_left.setPower(-x);
        front_right.setPower(x);
    }
    private void rotateLeft(double x){
        back_left.setPower(x);
        back_right.setPower(-x);
        front_left.setPower(x);
        front_right.setPower(-x);
    }
    private void rotateRight(double x){
        back_left.setPower(-x);
        back_right.setPower(x);
        front_left.setPower(-x);
        front_right.setPower(x);
    }
}