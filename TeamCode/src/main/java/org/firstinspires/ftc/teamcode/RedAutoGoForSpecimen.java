package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;



@Autonomous
public class RedAutoGoForSpecimen extends LinearOpMode {
    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;

    DcMotorEx rightSlideMotor;
    DcMotor leftSlideMotor;

    List<DcMotorEx> motors;


    IMU imu;


    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;
    public static final double NEW_F = 10;

    @Override
    public void runOpMode() throws InterruptedException {


        // Creates a PIDFController with gains kP, kI, kD, and kF
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);


        frontLeftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "frontLeftWheel");
        backLeftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "backLeftWheel");
        frontRightMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "frontRightWheel");
        backRightMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "backRightWheel");

        rightSlideMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlideMotor = hardwareMap.dcMotor.get("leftSlide");

        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors = new ArrayList<>();

        motors.add(frontLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backLeftMotor);
        motors.add(backRightMotor);

        telemetry.addData("frontLeftMotor", frontLeftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("frontRightMotor", frontRightMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("backLeftMotor", backLeftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("backRightMotor", backRightMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

        telemetry.addData("rightSlideMotor", rightSlideMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        imu.resetYaw();

        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        int desiredPosition = 0;

        gyroSensorOrientation orient = new gyroSensorOrientation();

        telemetry.update();

        waitForStart();

        leftSlideMotor.setTargetPosition(400);
        rightSlideMotor.setTargetPosition(-400);

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlideMotor.setPower(0.4);
        rightSlideMotor.setPower(0.4);

        telemetry.addData("Motors Forward","");
        telemetry.update();
        motors.forEach(motor -> {
            motor.setTargetPosition(-300);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.6);
        });



//        frontLeftMotor.setTargetPosition(desiredPosition);
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRightMotor.setTargetPosition(desiredPosition);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backLeftMotor.setTargetPosition(desiredPosition);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backRightMotor.setTargetPosition(desiredPosition);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        frontLeftMotor.setPower(0.6);
//        frontRightMotor.setPower(0.6);

//        while(frontRightMotor.isBusy() && frontLeftMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())
//        {
//            robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            telemetry.addData("list of motors" , motors);
//            telemetry.update();
//
//            orient.autoOrient(robotHeading, 0, 0.6, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);
//        }
//        motors.forEach(motor -> {
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            // motor.setPower(0);
//        });

        sleep(2500);


//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Motors Strafe","");
        telemetry.update();

/*
        boolean isFinished = false;
        while(!isFinished)
        {
            robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            orient.autoOrient(robotHeading, 90, 0, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);
            telemetry.update();

            if(Math.abs(Math.abs(robotHeading) - 90) < 0.5)
            {
                motors.forEach(motor -> {
                    motor.setTargetPosition(motor.getCurrentPosition());
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setPower(0);
                });

                isFinished = true;
            }
        }
*/


        strafe(3000, -1);
    }


    private void strafe(int desiredPosition, int direction) {

       /* for(int i = 50; i < desiredPosition; i+=50) {

            frontLeftMotor.setTargetPosition(i * direction);
            backRightMotor.setTargetPosition(i * direction);
            frontRightMotor.setTargetPosition(i * -direction);
            backLeftMotor.setTargetPosition(i * -direction);

            boolean isFinished = false;

            while(!isFinished)
            {
                telemetry.addData("trying to strafe", "");
                telemetry.update();
                  frontRightMotor.setPower(0.6);
                  backRightMotor.setPower(0.4);
                  frontLeftMotor.setPower(0.6);
                  frontLeftMotor.setPower(0.4);

                while(frontRightMotor.isBusy() && frontLeftMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())
                {

                }
                isFinished = true;
            }
        } */

        frontLeftMotor.setTargetPosition(desiredPosition * direction);
        backRightMotor.setTargetPosition(desiredPosition * direction);
        frontRightMotor.setTargetPosition(desiredPosition * -direction);
        backLeftMotor.setTargetPosition(desiredPosition * -direction);


            telemetry.addData("trying to strafe", "");
            telemetry.update();
            frontRightMotor.setPower(0.6);
            backRightMotor.setPower(0.2);
            frontLeftMotor.setPower(0.6);
            backLeftMotor.setPower(0.2);



            while (frontRightMotor.isBusy() && frontLeftMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
                telemetry.addData("back left motor position", backLeftMotor.getCurrentPosition());
                telemetry.update();
            }
    }
}
