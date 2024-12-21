package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class gyroSensorOrientation{
public static void autoOrient(double currentOrientation, double wantedOrientation, double defaultForce, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, DcMotor backLeftMotor, Telemetry telemetry) {
    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    if (currentOrientation > wantedOrientation)
    {
        frontLeftMotor.setPower(-.8d);
        backLeftMotor.setPower(-.8d);

        frontRightMotor.setPower(.8d);
        backRightMotor.setPower(.8d);
        telemetry.addData("turning Right", "");

    }

    else if(0.02d < Math.abs(Math.abs(wantedOrientation) - Math.abs(currentOrientation)))
    {
        telemetry.addData("normal", "");
        frontLeftMotor.setPower(defaultForce);
        backLeftMotor.setPower(defaultForce);

        frontRightMotor.setPower(defaultForce);
        backRightMotor.setPower(defaultForce);
    }
    if (currentOrientation < wantedOrientation)
    {
        telemetry.addData("turning Left", "");

        frontLeftMotor.setPower(.8d);
        backLeftMotor.setPower(.8d);

        frontRightMotor.setPower(-.8d);
        backRightMotor.setPower(-.8d);
    }
    telemetry.addData("Normal Calculation", Math.abs(Math.abs(wantedOrientation) - Math.abs(currentOrientation)));
    //telemetry.addData("on 360 base", -currentOrientation + 180);
    //telemetry.addData("Current Orientation", currentOrientation);
    //telemetry.addData("Wanted Orientation", wantedOrientation);
}

}
