package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.ArrayList;

@TeleOp
public class SampleDrive extends LinearOpMode{
    double startingRightBucketPos;
    double startingLeftBucketPos;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor bucketMotor = hardwareMap.dcMotor.get("bucketSlideMotor");

        DcMotor leftSlide = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlide = hardwareMap.dcMotor.get("rightSlide");

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CRServo leftSlideIntakeServo = hardwareMap.crservo.get("leftSlideIntakeServo");
        CRServo rightSlideIntakeServo = hardwareMap.crservo.get("rightSlideIntakeServo");

        Servo clawGrabServo = hardwareMap.servo.get("clawGrabServo");
        CRServo clawRotateServo = hardwareMap.crservo.get("clawRotateServo");
        Servo clawFlipServo = hardwareMap.servo.get("clawFlipServo");

        ArrayList<DcMotor> wheelMotors = new ArrayList<DcMotor>();

        wheelMotors.add(frontLeftMotor);
        wheelMotors.add(backLeftMotor);
        wheelMotors.add(frontRightMotor);
        wheelMotors.add(backRightMotor);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);


        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        double rotateTo = 0;
        boolean rightTriggerRotate = false;
        boolean leftTriggerRotate = false;


        startingRightBucketPos = bucketMotor.getCurrentPosition();


        boolean bumperHeld = false;
        boolean leftBumperHeld = false;

        double rightBumperPos = 0.6;
        double leftBumperPos = 0;

        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;

        double y;
        double x;
        double rx;

        boolean isStrafing = false;
        boolean lastIsStrafing = false;

        double targetOrientation = 0;

        gyroSensorOrientation gyroSensorOrientation1 = new gyroSensorOrientation();

        /// WHILE LOOOOOOOOOOOOOP
        while (opModeIsActive()) {
            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            final String driver = "meg";

            if(driver != "meg") {
                y = Math.abs(gamepad1.left_stick_y) > 0.1 ? gamepad1.left_stick_y : gamepad2.left_stick_y; // Remember, Y stick value is reversed
                x = Math.abs(gamepad1.left_stick_x) > 0.1 ? -gamepad1.left_stick_x * 1.1 : -gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
                rx = Math.abs(gamepad1.right_stick_x) > 0.1 ? gamepad1.right_stick_x : gamepad2.right_stick_x;
            }
            else
            {
                y = Math.abs(gamepad1.right_stick_y) > 0.1 ? gamepad1.right_stick_y : gamepad2.left_stick_y; // Remember, Y stick value is reversed
                x = Math.abs(gamepad1.right_stick_x) > 0.1 ? -gamepad1.right_stick_x * 1.1 : -gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
                rx = Math.abs(gamepad1.left_stick_x) > 0.1 ? gamepad1.left_stick_x : gamepad2.right_stick_x;
            }

            telemetry.addData("Driver", driver);

            boolean hasXVal = false;

            gyroSensorOrientation gyroSensorOrientation = new gyroSensorOrientation();

            telemetry.addData("x value ", x);
            telemetry.addData("y value ", y);

            telemetry.addData("rx value ", rx);

            telemetry.addData("gamepad 2 right stick x", gamepad2.right_stick_x);
            boolean aButtonDown = gamepad1.a;
            boolean bButtonDown = gamepad1.b;

            boolean rightBumper = gamepad2.left_bumper;
            boolean leftBumper = gamepad2.right_bumper;


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

            if((Math.abs(y) < 0.13))
            {
                y = 0;
            }
            if(Math.abs(x) < 0.13)
            {
                x = 0;
            }

            if(aButtonDown)
            {
                rx += 1;
            }
            if (bButtonDown)
            {
                rx -= 1;
            }

            leftSlideIntakeServo.setPower(0);
            rightSlideIntakeServo.setPower(0);

            float gP2RT = gamepad2.right_trigger;
            float gP2LT = gamepad2.left_trigger;
            boolean g2LTdown = false;
            boolean g2RTdown = false;

            if(gP2RT > 0.05) g2RTdown = true;
            if(gP2LT > 0.05) g2LTdown = true;

            if(g2RTdown)
            {
                leftSlideIntakeServo.setDirection(CRServo.Direction.REVERSE);
                rightSlideIntakeServo.setDirection(CRServo.Direction.FORWARD);
                leftSlideIntakeServo.setPower(gP2RT);
                rightSlideIntakeServo.setPower(gP2RT);
            }
            else if(!g2LTdown)
            {
                leftSlideIntakeServo.setPower(0);
                rightSlideIntakeServo.setPower(0);
            }
            if(g2LTdown)
            {
                leftSlideIntakeServo.setDirection(CRServo.Direction.FORWARD);
                rightSlideIntakeServo.setDirection(CRServo.Direction.REVERSE);
                leftSlideIntakeServo.setPower(gP2LT);
                rightSlideIntakeServo.setPower(gP2LT);
            }
            else if(!g2RTdown)
            {
                leftSlideIntakeServo.setPower(0);
                rightSlideIntakeServo.setPower(0);
            }



            if(Gp2XButtonDown)
            {
                clawRotateServo.setPower(-.1);
            }
            if(Gp2BButtonDown)
            {
                clawRotateServo.setPower(.1);
            }
            else if(!Gp2XButtonDown)
            {
                clawRotateServo.setPower(0);
            }

            //clawGrabServo
            telemetry.addData("claw grab pos", clawGrabServo.getPosition());
                //corner movement (front is top right corner) in comments below
                if (Math.abs(gamepad1.right_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
//                    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//                    if(Math.abs(x) > 0.5)
//                    {
//                        x = (x > 0 ? 0.5 : -0.5);
//                    }

                    //left wheel
                    frontLeftPower = ((y * 2) + (x) - rx);
                    //double frontLeftPower = (-x - rx);

                    //back wheel
                    backLeftPower = ((y * 2) - (x) - rx);
                    //double backLeftPower = (y - rx);

                    //front wheel
                    frontRightPower = ((y * 2) - (x) + rx);
                    //double frontRightPower = (y + rx);

                    //right wheel
                    backRightPower = ((y * 2) + (x) + rx);
                    //double backRightPower = (-x + rx);


                    //power (0-1)

                    final double z = 1;
                    frontLeftMotor.setPower(z * frontLeftPower);
                    backLeftMotor.setPower(z * backLeftPower);
                    frontRightMotor.setPower(z * frontRightPower);
                    backRightMotor.setPower(z * backRightPower);



                }
                else {

                    telemetry.addData("gamepad 2 is moving", "");

                    frontLeftPower = (y - x + (rx * 2));

                    //back wheel
                    backLeftPower = (y + x + (rx * 2));

                    //front wheel
                    frontRightPower = (y + x - (rx * 2));

                    //right wheel
                    backRightPower = (y - x - (rx * 2));

                    // power (0-1)
                    double z = 0.5;
                    frontLeftMotor.setPower(z * frontLeftPower);
                    backLeftMotor.setPower(z * backLeftPower);
                    frontRightMotor.setPower(z * frontRightPower);
                    backRightMotor.setPower(z * backRightPower);
                }

            Double TotalSpeed = x + y;
            telemetry.addData("X Speed: " + x, "");
            telemetry.addData("Y speed: " + y, "");
            telemetry.addData("Total Speed: " + TotalSpeed, "" );
            telemetry.addData("Rotation Speed: " + rx, "");


            if(rightTriggerDown)
            {
                rotateTo = BumperRotation.Rotate(robotHeading, "right", telemetry);
                rightTriggerRotate = true;
                //gyroSensorOrientation.autoOrient(robotHeading, BumperRotation.Rotate(robotHeading, "right", telemetry), .1, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);
                //telemetry.addData("Bumper Rotation Angle",BumperRotation.Rotate(robotHeading, "right", telemetry));
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
            if(leftTriggerRotate)
            {
                telemetry.addData("remainder", 45 - Math.abs(robotHeading) % 45);
                gyroSensorOrientation.autoOrient(robotHeading, rotateTo, 0, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);
                telemetry.addData("Bumper Rotation Angle",rotateTo);
                rightTriggerRotate = BumperRotation.stopAtClosestInterval(robotHeading, telemetry);
            }

            if(dPadUp)
            {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 50);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 50);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }
            if(dPadDown)
            {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 50);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 50);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }

            if(Gp2DPadUp)
            {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 1000);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 1000);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }
            if(Gp2DPadDown)
            {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 1000);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 1000);
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
            if(Gp2YButtonDown)
            {
                bucketMotor.setTargetPosition(bucketMotor.getCurrentPosition() + 100);
                bucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bucketMotor.setPower(0.6);
            }
            if(Gp2AButtonDown) {
                bucketMotor.setTargetPosition(bucketMotor.getCurrentPosition() - 100);
                bucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bucketMotor.setPower(0.6);
            }
            if(!Gp2AButtonDown && !Gp2YButtonDown) {
                bucketMotor.setTargetPosition(bucketMotor.getCurrentPosition());
                bucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bucketMotor.setPower(0.0);
            }

            if(rightBumper && !bumperHeld)
            {
                telemetry.addData("right bumper pressed", "");
                if(rightBumperPos == 0.2)
                {
                    rightBumperPos = .6;
                }
                else if(rightBumperPos == .6)
                {
                    rightBumperPos = 0.2;
                }
            }

            if(leftBumper && !leftBumperHeld)
            {
                telemetry.addData("leftBumperPressed", leftBumperPos);
                if(leftBumperPos == 0)
                {
                    leftBumperPos = 0.8;
                    telemetry.addData("Left Bumper Pos set to 1", "");
                }
                else if(leftBumperPos == 0.8)
                {
                    leftBumperPos = 0;
                }
            }



            final double z = 1;


            telemetry.addData("right side pos", rightSlide.getCurrentPosition());
            telemetry.addData("left side pos", leftSlide.getCurrentPosition());

            telemetry.addData("backLeftMotorPower", backLeftMotor.getPower());
            telemetry.addData("backRightMotorPower", backRightMotor.getPower());
            telemetry.addData("frontRightMotorPower", frontRightMotor.getPower());
            telemetry.addData("frontLeftMotorPower", frontLeftMotor.getPower());

            telemetry.update();

            clawGrabServo.setPosition(rightBumperPos);
            clawFlipServo.setPosition(leftBumperPos);

            bumperHeld = rightBumper;
            leftBumperHeld = leftBumper;
        }
    }
}
