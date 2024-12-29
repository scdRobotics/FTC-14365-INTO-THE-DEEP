package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class RedAutoGoForSpecimen extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    List<DcMotor> motors;
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftWheel");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftWheel");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightWheel");
        backRightMotor = hardwareMap.dcMotor.get("backRightWheel");

        motors = new ArrayList<>();

        motors.add(frontLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backLeftMotor);
        motors.add(backRightMotor);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        imu.resetYaw();

        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        int desiredPosition = 0;

        gyroSensorOrientation orient = new gyroSensorOrientation();

        waitForStart();


        motors.forEach(motor -> {
            motor.setTargetPosition(100);
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

        while(frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())
        {
            robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("list of motors" , motors);
            telemetry.update();

            orient.autoOrient(robotHeading, 0, 0.6, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);
        }
        motors.forEach(motor -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // motor.setPower(0);
        });

        desiredPosition = 1000;
        /// refactor to use strafe function
        frontLeftMotor.setTargetPosition(-desiredPosition);
        frontRightMotor.setTargetPosition(desiredPosition);
        backRightMotor.setTargetPosition(-desiredPosition);
        backLeftMotor.setTargetPosition(desiredPosition);

        motors.forEach(motor -> {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.5);
        });

        while(frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())
        {
            robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("robot heading", robotHeading);
            telemetry.update();
            orient.autoOrient(robotHeading, 0, 0.5, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);
        }
    }



    private void strafe(int desiredPosition) {
        frontLeftMotor.setTargetPosition(-desiredPosition);
        frontRightMotor.setTargetPosition(desiredPosition);
        backRightMotor.setTargetPosition(-desiredPosition);
        backLeftMotor.setTargetPosition(desiredPosition);

        motors.forEach(motor -> {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.5);
        });
    }
}
