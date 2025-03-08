package org.firstinspires.ftc.teamcode;

import android.bluetooth.BluetoothClass;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class TurnTestAuto extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        //
        //Declare Motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        CRServo leftSlideIntakeServo = hardwareMap.crservo.get("leftSlideIntakeServo");
        CRServo rightSlideIntakeServo = hardwareMap.crservo.get("rightSlideIntakeServo");


        // CRServo servo = hardwareMap.crservo.get("servo");

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        imu.resetYaw();

        gyroSensorOrientation gyroSensorOrientation = new gyroSensorOrientation();

        Double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addData("robot Heading", robotHeading);
        telemetry.update();

        waitForStart();

        strafe(1000);
        turn(-30);
        strafe(1000);

    }

    void turn(double desiredOrientation)
    {
        double robotHeading;

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        long startTime = System.currentTimeMillis();

        while(true)
        {

            robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            gyroSensorOrientation.autoOrient(robotHeading, desiredOrientation, 0, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);

            if (gyroSensorOrientation.phase == "normal")
            {
                break;
            }

            if(System.currentTimeMillis() - startTime > 3000)
            {
                break;
            }
            telemetry.addData("turn time",  System.currentTimeMillis() - startTime);
            telemetry.addData("time left", 3000 - (System.currentTimeMillis() - startTime));

            telemetry.addData("robotHeading", robotHeading);
        }
    }

    void strafe(int desiredPosition)
    {
        double robotHeading;

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(frontLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backLeftMotor);
        motors.add(backRightMotor);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        frontLeftMotor.setTargetPosition(desiredPosition);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower(0.4);

        backLeftMotor.setTargetPosition(-desiredPosition);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setPower(0.4);

        frontRightMotor.setTargetPosition(desiredPosition);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setPower(0.4);

        backRightMotor.setTargetPosition(-desiredPosition);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setPower(0.4);

        telemetry.addData("Is Strafing", "");
        telemetry.update();

        while(true)
        {
            double frontRight = Math.abs(frontRightMotor.getCurrentPosition());
            double frontLeft = Math.abs(frontLeftMotor.getCurrentPosition());
            double backRight = Math.abs(backRightMotor.getCurrentPosition());
            double backLeft = Math.abs(backLeftMotor.getCurrentPosition());
            double desiredPos = Math.abs(desiredPosition);

            double currentOrientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if(frontRight - desiredPos > 5 && frontLeft - desiredPos > 5 && backRight - desiredPos > 5 && backLeft - desiredPos > 5)
            {
                break;
            }
        }
    }


    void forward(int desiredPosition, double power)
    {
        boolean isFinished = false;
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(frontLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backLeftMotor);
        motors.add(backRightMotor);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            motor.setTargetPosition(-desiredPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }

        while(true)
        {
            telemetry.addData("frontRightMotor position", frontRightMotor.getCurrentPosition());
            telemetry.addData("frontRightMotor desired position", frontRightMotor.getTargetPosition());
            telemetry.update();

            if(Math.abs(Math.abs(frontRightMotor.getCurrentPosition()) - Math.abs(frontRightMotor.getTargetPosition())) < 5)
            {
                break;
            }
            if(Math.abs(Math.abs(frontLeftMotor.getCurrentPosition()) - Math.abs(frontLeftMotor.getTargetPosition())) < 5)
            {
                break;
            }

        }

    }

    void backwards(int desiredPosition)
    {
        boolean isFinished = false;
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(frontLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backLeftMotor);
        motors.add(backRightMotor);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            motor.setTargetPosition(desiredPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.6);
        }

        while(true)
        {
            telemetry.addData("frontRightMotor position", frontRightMotor.getCurrentPosition());
            telemetry.addData("frontRightMotor desired position", frontRightMotor.getTargetPosition());
            telemetry.update();

            if(Math.abs(Math.abs(frontRightMotor.getCurrentPosition()) - Math.abs(frontRightMotor.getTargetPosition())) < 5)
            {
                break;
            }
            if(Math.abs(Math.abs(frontLeftMotor.getCurrentPosition()) - Math.abs(frontLeftMotor.getTargetPosition())) < 5)
            {
                break;
            }
        }
    }

    void moveSlide(int desiredPosition)
    {
        DcMotor leftSlideMotor = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlideMotor = hardwareMap.dcMotor.get("rightSlide");

        CRServo leftSlideIntakeServo = hardwareMap.crservo.get("leftSlideIntakeServo");
        CRServo rightSlideIntakeServo = hardwareMap.crservo.get("rightSlideIntakeServo");

        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        ArrayList<CRServo> servos = new ArrayList<CRServo>();

        motors.add(leftSlideMotor);
        motors.add(rightSlideMotor);

        servos.add(leftSlideIntakeServo);
        servos.add(rightSlideIntakeServo);

        for(DcMotor motor : motors)
        {

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            motor.setTargetPosition(desiredPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
        if(desiredPosition < 0)
        {
            leftSlideIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

            for(CRServo servo : servos)
            {
                servo.setPower(1);
            }
        }


        while(true)
        {
            telemetry.addData("RightSlideMotor position", rightSlideMotor.getCurrentPosition());
            telemetry.addData("LeftSlideMotor position", leftSlideMotor.getCurrentPosition());


            telemetry.addData("rightSideDistance", Math.abs(Math.abs(rightSlideMotor.getCurrentPosition()) - Math.abs(rightSlideMotor.getTargetPosition())));
            telemetry.addData("leftSideDistance", Math.abs(Math.abs(leftSlideMotor.getCurrentPosition()) - Math.abs(leftSlideMotor.getTargetPosition())));
            telemetry.update();

            if(Math.abs(Math.abs(rightSlideMotor.getCurrentPosition()) - Math.abs(rightSlideMotor.getTargetPosition())) < 40)
            {
                for(DcMotor motor : motors)
                {
                    motor.setTargetPosition(motor.getCurrentPosition());
                }
                for(CRServo servo : servos)
                {
                    servo.setPower(0);
                }
                break;
            }
            if(Math.abs(Math.abs(leftSlideMotor.getCurrentPosition()) - Math.abs(leftSlideMotor.getTargetPosition())) < 40)
            {
                for(DcMotor motor : motors)
                {
                    motor.setTargetPosition(motor.getCurrentPosition());
                }
                for(CRServo servo : servos)
                {
                    servo.setPower(0);
                }
                break;
            }

        }
    }


}



