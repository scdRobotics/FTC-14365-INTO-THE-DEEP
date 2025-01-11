package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class gyroSensorOrientation{

    public static void autoOrient(double currentOrientation, double wantedOrientation, double defaultForce, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, DcMotor backLeftMotor, Telemetry telemetry)
    {
        autoOrient(currentOrientation, wantedOrientation, defaultForce, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry, true,true,true,true);
    }
public static void autoOrient(double currentOrientation, double wantedOrientation, double defaultForce, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, DcMotor backLeftMotor, Telemetry telemetry, boolean usingFrontRight, boolean usingFrontLeft, boolean usingBackRight, boolean usingBackLeft) {
    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    List<DcMotor> motors;

    motors = new ArrayList<>();

    motors.add(frontLeftMotor);
    motors.add(frontRightMotor);
    motors.add(backLeftMotor);
    motors.add(backRightMotor);


DcMotorSimple.Direction startingDirectionRightMotor = frontRightMotor.getDirection();



    if (currentOrientation > wantedOrientation)
    {
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        motors.forEach(motor -> {
            motor.setTargetPosition(motor.getTargetPosition() != 0 ? motor.getTargetPosition() : motor.getCurrentPosition() + 50);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        });

        frontLeftMotor.setPower(.8d);
        backLeftMotor.setPower(0);

        frontRightMotor.setPower(.8d);
        backRightMotor.setPower(0);
        telemetry.addData("turning Right", currentOrientation);

    }

    else if(0.02d < Math.abs(Math.abs(wantedOrientation) - Math.abs(currentOrientation)))
    {
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("normal", currentOrientation);
        if(usingFrontLeft) frontLeftMotor.setPower(defaultForce);
        if(usingBackLeft) backLeftMotor.setPower(defaultForce);

        if(usingFrontRight) frontRightMotor.setPower(defaultForce);
        if(usingBackRight) backRightMotor.setPower(defaultForce);
    }
    else if (currentOrientation < wantedOrientation)
    {
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.forEach(motor -> {
            motor.setTargetPosition(motor.getTargetPosition() != 0 ? motor.getTargetPosition() : motor.getCurrentPosition() + 50);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        });

        telemetry.addData("turning Left", currentOrientation);

        frontLeftMotor.setPower(.8d);
        backLeftMotor.setPower(0);

        frontRightMotor.setPower(.8d);
        backRightMotor.setPower(0);
    }



    telemetry.addData("Normal Calculation", Math.abs(Math.abs(wantedOrientation) - Math.abs(currentOrientation)));
    //telemetry.addData("on 360 base", -currentOrientation + 180);
    telemetry.addData("Current Orientation", currentOrientation);
    telemetry.addData("Wanted Orientation", wantedOrientation);

}

}
