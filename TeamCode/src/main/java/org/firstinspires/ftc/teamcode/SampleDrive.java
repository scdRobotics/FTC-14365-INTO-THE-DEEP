package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp
public class SampleDrive extends LinearOpMode{

    int cameraMonitorViewId;
    //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    OpenCvCamera camera;// = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), 1);
    VisionPipeline visionPipeline = new VisionPipeline();
    public void activateYellowPipelineCamera1(){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        camera.setPipeline(visionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera Opened! ", "");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }
    @Override
    public void runOpMode() throws InterruptedException {

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);


        AprilTagProcessor myAprilTagProcessor;

        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        VisionPortal portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), myAprilTagProcessor);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera openCvCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        double rotateTo = 0;
        imu.resetYaw();
        boolean rightTriggerRotate = false;
        boolean leftTriggerRotate = false;

        // WHILE LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP
        while (opModeIsActive()) {

            telemetry.addData("1 = working", visionPipeline.getI());
            openCvCamera.setPipeline(visionPipeline);
            telemetry.addData("webcam", portal.getCameraState());
            for (AprilTagDetection aprilTagDetection : myAprilTagProcessor.getDetections()) {
                telemetry.addData("april tag id", aprilTagDetection.id);
                telemetry.addData("decisionMargin", aprilTagDetection.decisionMargin);
            }
            telemetry.addData("rectangle", visionPipeline.getRect());
            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            telemetry.addData("Robot Heading", robotHeading);

           // boolean invertControls = false;

            // Toggle Arcade Style controls. Leave false for tank style.
            boolean ArcadeStyle = false;

            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            boolean aButtonDown = gamepad1.a;
            boolean bButtonDown = gamepad1.b;
            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger = gamepad1.left_trigger;
            boolean rightTriggerDown;
            if(rightTrigger > .5d) rightTriggerDown = true;
            else rightTriggerDown = false;
            boolean leftTriggerDown;
            if(leftTrigger > .5d) leftTriggerDown = true;
            else leftTriggerDown = false;





            /*if(invertControls)
            {
                x = -x;
                y = -y;
                rx = -rx;
            }*/

            // change this value to adjust max speed
            double maxSpeed = 1;
            // change these values to change max speed in specific directions. Keep same as maxSpeed for no impact
            double backwardsMaxSpeed = 1;
            double forwardMaxSpeed = 1;
            double rightMaxSpeed = 1;
            double leftMaxSpeed = 1;

            if((Math.abs(y) < 0.13))
            {
                telemetry.addData("In Deadzone", "");
                y = 0;
            }
            if(Math.abs(x) < 0.13)
            {
                telemetry.addData("In Deadzone", "");
                x = 0;
            }

            //if(Math.abs(x) > Math.abs(y)) y=0; else x = 0;

            //robot only runs at max speed
            /*if (y != 0)
            {
                if(y > 0)
                {
                    y = 1;
                }
                else
                {
                    y = -1;
                }
            }

            if(x != 0)
            {
                if (x > 0)
                {
                    x = 1;
                }
                if(x < 0) {
                    x = -1;
                }
            }*/

            if(Math.abs(x) > maxSpeed)
            {
                if(x < 0) x = -maxSpeed;
                else x = maxSpeed;
            }
            if(Math.abs(y) > maxSpeed)
            {
                if(y < 0) y = -maxSpeed;
                else y = maxSpeed;
            }

            if(x > forwardMaxSpeed) x = forwardMaxSpeed;
            if(x < -backwardsMaxSpeed) x = -backwardsMaxSpeed;
            if(y > rightMaxSpeed) y = rightMaxSpeed;
            if(y < -leftMaxSpeed) y = -leftMaxSpeed;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            // x = x * 1.5d;

            // random challenge thing
            // if(ArcadeStyle) rx = 0;


            if(aButtonDown)
            {
                rx += 1;
                telemetry.addData("abuttondown", "");
            }
            if (bButtonDown
            ) {

                rx -= 1;
                telemetry.addData("b Button Down", "");
            }


            //corner movement (front is top right corner) in comments below

            //left wheel
            double frontLeftPower = (y - x - rx);
            //double frontLeftPower = (-x - rx);

            //back wheel
            double backLeftPower = (y + x - rx);
            //double backLeftPower = (y - rx);

            //front wheel
            double frontRightPower = (y + x + rx);
            //double frontRightPower = (y + rx);

            //right wheel
            double backRightPower = (y - x + rx);
            //double backRightPower = (-x + rx);



            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            Double TotalSpeed = x + y;
            telemetry.addData("X Speed: " + x, "");
            telemetry.addData("Y speed: " + y, "");
            telemetry.addData("Total Speed: " + TotalSpeed, "" );
            telemetry.addData("Rotation Speed: " + rx, "");
            telemetry.update();


            if(rightTriggerDown)
            {
                rotateTo = BumperRotation.Rotate(robotHeading, "right", telemetry);
                rightTriggerRotate = true;
               // gyroSensorOrientation.autoOrient(robotHeading, BumperRotation.Rotate(robotHeading, "right", telemetry), .1, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);
               // telemetry.addData("Bumper Rotation Angle",BumperRotation.Rotate(robotHeading, "right", telemetry));
            }
            if(leftTriggerDown)
            {
                rotateTo = BumperRotation.Rotate(robotHeading, "left", telemetry);
                leftTriggerRotate = true;
                gyroSensorOrientation.autoOrient(robotHeading, BumperRotation.Rotate(robotHeading, "left", telemetry), 0, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);
                telemetry.addData("Bumper Rotation Angle",BumperRotation.Rotate(robotHeading, "left", telemetry));
            }
            if(rightTriggerRotate)
            {
                telemetry.addData("remainder", 45 - Math.abs(robotHeading) % 45);
                gyroSensorOrientation.autoOrient(robotHeading, rotateTo, 0, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);
                telemetry.addData("Bumper Rotation Angle",rotateTo);
                rightTriggerRotate = BumperRotation.stopAtClosestInterval(robotHeading, telemetry);
            }
            if(leftTriggerDown)
            {
                telemetry.addData("remainder", 45 - Math.abs(robotHeading) % 45);
                gyroSensorOrientation.autoOrient(robotHeading, rotateTo, 0, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);
                telemetry.addData("Bumper Rotation Angle",rotateTo);
                rightTriggerRotate = BumperRotation.stopAtClosestInterval(robotHeading, telemetry);
            }
        }
    }
    public void WriteTelemetry(String messageCaption,String messageValue)
    {
        telemetry.addData(messageCaption, messageValue);
        telemetry.update();
    }

}
