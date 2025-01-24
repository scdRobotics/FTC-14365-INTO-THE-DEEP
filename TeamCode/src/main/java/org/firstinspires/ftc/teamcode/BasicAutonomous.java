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
public class BasicAutonomous extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        //
        //Declare Motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftWheel");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftWheel");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightWheel");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightWheel");

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

        boolean isRunning = false;

        forward(900, 0.6);

        sleep(100);

        moveSlide(2800);

        forward(300, 0.3);

        leftSlideIntakeServo.setDirection(CRServo.Direction.FORWARD);
        rightSlideIntakeServo.setDirection(CRServo.Direction.REVERSE);

        rightSlideIntakeServo.setPower(1);
        leftSlideIntakeServo.setPower(1);

        moveSlide(-500);

        rightSlideIntakeServo.setPower(1);
        leftSlideIntakeServo.setPower(1);

        moveSlide(-700);

        rightSlideIntakeServo.setPower(0);
        leftSlideIntakeServo.setPower(0);

        backwards(650);

        turn(-90, imu);

        forward(1400, 0.6);

        turn(-180, imu);

        moveSlide(-1400);

        forward(1225, 0.4);

        //turn(0, imu);

    }

    void turn(double desiredOrientation, IMU imu)
    {
        double robotHeading;

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftWheel");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftWheel");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightWheel");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightWheel");

        while(true)
        {
            robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            gyroSensorOrientation.autoOrient(robotHeading, desiredOrientation, 0, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);

            if(gyroSensorOrientation.phase == "normal")
            {
                break;
            }
        }
    }

    void forward(int desiredPosition, double power)
    {
        boolean isFinished = false;
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftWheel");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftWheel");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightWheel");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightWheel");

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

        }

    }

    void backwards(int desiredPosition)
    {
        boolean isFinished = false;
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftWheel");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftWheel");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightWheel");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightWheel");

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

        }
    }

    void moveSlide(int desiredPosition)
    {
        DcMotor leftSlideMotor = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlideMotor = hardwareMap.dcMotor.get("rightSlide");

        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();

        motors.add(leftSlideMotor);
        motors.add(rightSlideMotor);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            motor.setTargetPosition(desiredPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }


        while(true)
        {
            telemetry.addData("RightSlideMotor position", rightSlideMotor.getCurrentPosition());
            telemetry.addData("RightslideMotor desired position", rightSlideMotor.getTargetPosition());
            telemetry.update();

            if(Math.abs(Math.abs(rightSlideMotor.getCurrentPosition()) - Math.abs(rightSlideMotor.getTargetPosition())) < 40)
            {
                for(DcMotor motor : motors)
                {
                    motor.setTargetPosition(motor.getCurrentPosition());
                }
                break;
            }

        }
    }


}



