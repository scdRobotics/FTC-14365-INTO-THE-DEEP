package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.YellowVisionPipeline.boundingRect;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.openftc.easyopencv.OpenCvCameraBase;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
@TeleOp
public class SampleDrive extends LinearOpMode{

    //VisionPipeline visionPipeline = new VisionPipeline();
    //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    //public OpenCvCamera camera;
    boolean cameraEnabled;

    public void closeYellowPipelineCamera1(OpenCvCamera camera)
    {
        camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                cameraEnabled = false;
                //camera.startStreaming(1280,720,OpenCvCameraRotation.UPRIGHT);
                //camera.stopStreaming();
            }
        });
    }

    public void activateYellowPipelineCamera1(OpenCvCamera camera){
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {

            @Override
            public void onOpened()
            {

                cameraEnabled = true;
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
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

        YellowVisionPipeline yellowVisionPipeline = new YellowVisionPipeline();
        BlueVisionPipeline blueVisionPipeline = new BlueVisionPipeline();
        RedVisionPipeline redVisionPipeline = new RedVisionPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //telemetry.addData("camera", camera);
        activateYellowPipelineCamera1(camera);
        camera.setPipeline(yellowVisionPipeline);
        sleep(1000);
        closeYellowPipelineCamera1(camera);
        Rect rect = yellowVisionPipeline.getRect();
        telemetry.addData("rect", rect);


        //AprilTagProcessor myAprilTagProcessor;

        //myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftWheel");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftWheel");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightWheel");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightWheel");

        DcMotor leftSlide = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlide = hardwareMap.dcMotor.get("rightSlide");

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CRServo leftSlideIntakeServo = hardwareMap.crservo.get("leftSlideIntakeServo");
        CRServo rightSlideIntakeServo = hardwareMap.crservo.get("rightSlideIntakeServo");

        Servo bucketRotateServo = hardwareMap.servo.get("bucketServo");
        CRServo bucketIntakeServo = hardwareMap.crservo.get("bucketIntakeServo");

        //VisionPortal portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), myAprilTagProcessor);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        IMU imu = hardwareMap.get(IMU.class, "imu");
        //ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

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
        boolean isOpened = false;
        int i = 0;

        /// WHILE LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP

        boolean yPressedLastFrame = false;
        while (opModeIsActive()) {

            if(yellowVisionPipeline.getycbcrEdge() == null) telemetry.addData("Mat is null", "");
            else telemetry.addData("Mat is not null", "");


            //telemetry.addData("webcam", portal.getCameraState());
            /*for (AprilTagDetection aprilTagDetection : myAprilTagProcessor.getDetections()) {
                telemetry.addData("april tag id", aprilTagDetection.id);
                telemetry.addData("decisionMargin", aprilTagDetection.decisionMargin);
            }*/
            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            telemetry.addData("Robot Heading", robotHeading);

           // boolean invertControls = false;

            // Toggle Arcade Style controls. Leave false for tank style.
            boolean ArcadeStyle = false;

            int lastDPadUsed;

            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            boolean yButtonDown = gamepad1.y;
            boolean xButtonDown = gamepad1.x;
            boolean aButtonDown = gamepad1.a;
            boolean bButtonDown = gamepad1.b;
            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger = gamepad1.left_trigger;
            boolean rightTriggerDown;

            boolean dPadUp = gamepad1.dpad_up;
            boolean dPadDown = gamepad1.dpad_down;

            boolean Gp2AButtonDown = gamepad2.a;
            boolean Gp2BButtonDown = gamepad2.b;

            boolean Gp2YButtonDown = gamepad2.y;
            boolean Gp2XButtonDown = gamepad2.x;

            boolean Gp2DPadUp = gamepad2.dpad_up;
            boolean Gp2DPadDown = gamepad2.dpad_down;

            if(rightTrigger > .5d) rightTriggerDown = true;
            else rightTriggerDown = false;
            boolean leftTriggerDown;
            if(leftTrigger > .5d) leftTriggerDown = true;
            else leftTriggerDown = false;
            if(yButtonDown && !cameraEnabled)
            {
                activateYellowPipelineCamera1(camera);
                camera.setPipeline(yellowVisionPipeline);
            }
            if(yButtonDown && cameraEnabled && !yPressedLastFrame)
            {
                i++;
                if(i == 0)
                {
                    camera.setPipeline(yellowVisionPipeline);
                }
                if(i == 1)
                {
                    camera.setPipeline(blueVisionPipeline);
                }
                if(i == 2)
                {
                    camera.setPipeline(redVisionPipeline);
                }
                if(i == 3)
                {
                    i = 0;
                    camera.setPipeline(yellowVisionPipeline);
                }
            }
            if(xButtonDown && cameraEnabled) {
                closeYellowPipelineCamera1(camera);
                camera.setPipeline(null);
            }
            int rectX = 0;
            yPressedLastFrame = yButtonDown;
            if(cameraEnabled)
            {
                if(i == 0)
                {
                    rect = yellowVisionPipeline.getRect();
                    rectX = yellowVisionPipeline.getRectX();
                }
                if(i == 1)
                {
                    rect= blueVisionPipeline.getRect();
                    rectX = blueVisionPipeline.getRectX();
                }
                if(i == 2)
                {
                    rect = redVisionPipeline.getRect();
                    rectX = redVisionPipeline.getRectX();

                }

            }

            leftSlide.setPower(1);
            rightSlide.setPower(1);

            if(rect != null){
                telemetry.addData(" Rect", rect);

                telemetry.addData("Rect X value", rectX);
                telemetry.addData("Rect Y Value", rect.y);
            }

            telemetry.addData("i = ", i);
            /*if(invertControls)
            {
                x = -x;
                y = -y;
                rx = -rx;
            }*/

            // change this value to adjust max speed
            double maxSpeed = .5;
            // change these values to change max speed in specific directions. Keep same as maxSpeed for no impact
            double backwardsMaxSpeed = .5;
            double forwardMaxSpeed = .5;
            double rightMaxSpeed = .5;
            double leftMaxSpeed = .5;

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

            leftSlideIntakeServo.setPower(0);
            rightSlideIntakeServo.setPower(0);

            if(Gp2AButtonDown)
            {
                leftSlideIntakeServo.setDirection(CRServo.Direction.FORWARD);
                rightSlideIntakeServo.setDirection(CRServo.Direction.REVERSE);
                leftSlideIntakeServo.setPower(0.3);
                rightSlideIntakeServo.setPower(0.3);
            }
            else if(!Gp2BButtonDown)
            {
                leftSlideIntakeServo.setPower(0);
                rightSlideIntakeServo.setPower(0);
            }
            if(Gp2BButtonDown)
            {
                leftSlideIntakeServo.setDirection(CRServo.Direction.REVERSE);
                rightSlideIntakeServo.setDirection(CRServo.Direction.FORWARD);
                leftSlideIntakeServo.setPower(0.3);
                rightSlideIntakeServo.setPower(0.3);
            }
            else if(!Gp2AButtonDown)
            {
                leftSlideIntakeServo.setPower(0);
                rightSlideIntakeServo.setPower(0);
            }
            telemetry.addData("GamePad 2 a down", Gp2AButtonDown);
            telemetry.addData("GamePad 2 b down", Gp2BButtonDown);

            bucketIntakeServo.setPower(0);

            if(Gp2YButtonDown)
            {
                bucketIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
                bucketIntakeServo.setPower(0.3);
            }
            if(Gp2XButtonDown)
            {
                bucketIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
                bucketIntakeServo.setPower(0.3);
            }


            // 1 = idle position, 0.4-5 = collect position
            /*if(Gp2DPadUp)
            {
                bucketRotateServo.setPosition(1d);
            }
            if(Gp2DPadDown)
            {
                bucketRotateServo.setPosition(0.45d);
            }
            */

           // bucketRotateServo.
            telemetry.addData("Bucket Pos", bucketRotateServo.getPosition());


            //corner movement (front is top right corner) in comments below

            //left wheel
            double frontLeftPower = -(y + x + rx);
            //double frontLeftPower = (-x - rx);

            //back wheel
            double backLeftPower = -(y - x + rx);
            //double backLeftPower = (y - rx);

            //front wheel
            double frontRightPower = -(y - x - rx);
            //double frontRightPower = (y + rx);

            //right wheel
            double backRightPower = -(y + x - rx);
            //double backRightPower = (-x + rx);

            // power (0-1)
            double z = 1;
            frontLeftMotor.setPower(z * frontLeftPower);
            backLeftMotor.setPower(z * backLeftPower);
            frontRightMotor.setPower(z * frontRightPower);
            backRightMotor.setPower(z * backRightPower);

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

            if(dPadUp)
            {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 50);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 50);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }
            if(dPadDown)
            {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 50);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 50);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }

            telemetry.addData("Dpad Down", dPadDown);
            telemetry.addData("Dpad Up", dPadUp);
            if(Gp2DPadUp)
            {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 1000);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 1000);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }
            if(Gp2DPadDown)
            {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 1000);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 1000);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }
            if(!Gp2DPadDown && !Gp2DPadUp && !dPadUp && !dPadDown)
            {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition());
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition());
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            telemetry.addData("right side pos", rightSlide.getCurrentPosition());
            telemetry.addData("left side pos", leftSlide.getCurrentPosition());

            telemetry.addData("backRightPosition", backRightMotor.getCurrentPosition());
            telemetry.addData("backLeftPosition", backLeftMotor.getCurrentPosition());
            telemetry.addData("frontRightPosition", frontRightMotor.getCurrentPosition());
            telemetry.addData("frontLeftPosition", frontLeftMotor.getCurrentPosition());

        } //End of while loop

        activateYellowPipelineCamera1(camera);
        //camera.closeCameraDevice();
    }


}
