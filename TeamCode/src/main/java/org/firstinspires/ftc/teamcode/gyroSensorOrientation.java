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

    static String phase = "";
    public static void autoOrient(double currentOrientation, double wantedOrientation, double defaultForce, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, DcMotor backLeftMotor, Telemetry telemetry, boolean usingFrontRight, boolean usingFrontLeft, boolean usingBackRight, boolean usingBackLeft)
    {
        autoOrient(currentOrientation, wantedOrientation, defaultForce, defaultForce, defaultForce, defaultForce, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry, usingFrontRight, usingFrontLeft, usingBackRight, usingBackLeft);
    }
    public static void autoOrient(double currentOrientation, double wantedOrientation, double defaultForce, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, DcMotor backLeftMotor, Telemetry telemetry)
    {
        autoOrient(currentOrientation, wantedOrientation, defaultForce, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry, true,true,true,true);
    }
    public static void autoOrient(double currentOrientation, double wantedOrientation, double FLdefaultForce, double FRdefaultForce, double BLdefaultForce, double BRdefaultForce, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, DcMotor backLeftMotor, Telemetry telemetry, boolean usingFrontRight, boolean usingFrontLeft, boolean usingBackRight, boolean usingBackLeft) {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("runningAutoOrient", "");

            List<DcMotor> motors;

            motors = new ArrayList<>();

            motors.add(frontLeftMotor);
            motors.add(frontRightMotor);
            motors.add(backLeftMotor);
            motors.add(backRightMotor);




        motors.forEach(motor -> {
            if(motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        });


        double wantedDegrees = wantedOrientation + 180;
        double currentDegrees = currentOrientation + 180;

        double distance = Math.abs(currentDegrees - wantedDegrees);
        double distanceMagnitude = distance / 3.6;

        double power = 0.6 * distanceMagnitude;

        telemetry.addData("Distance from target orientation", distance);
        telemetry.addData("Distance Magnitude", distanceMagnitude);
        telemetry.addData("power", power);
        telemetry.update();

        FLdefaultForce = -FLdefaultForce;

        if (currentOrientation < wantedOrientation)
        {
            frontLeftMotor.setPower(Math.abs(FLdefaultForce + power));
            backLeftMotor.setPower(Math.abs(BLdefaultForce + power));

            frontRightMotor.setPower(-Math.abs(FRdefaultForce + power));
            backRightMotor.setPower(-Math.abs(BRdefaultForce + power));
            telemetry.addData("turning Left", currentOrientation);
            phase = "turning left";

        }

        else if(5d > Math.abs(Math.abs(wantedOrientation) - Math.abs(currentOrientation)))
        {
            telemetry.addData("normal", currentOrientation);
            if(usingFrontLeft) frontLeftMotor.setPower(FLdefaultForce);


            if(usingBackLeft) backLeftMotor.setPower(BLdefaultForce);


            if(usingFrontRight) frontRightMotor.setPower(FRdefaultForce);

            if(usingBackRight) backRightMotor.setPower(BRdefaultForce);

            phase = "normal";

        }
        else if (currentOrientation > wantedOrientation) {
            telemetry.addData("turning Right", currentOrientation);

            frontLeftMotor.setPower(-Math.abs(FLdefaultForce + power));
            backLeftMotor.setPower(-Math.abs(BLdefaultForce + power));

            frontRightMotor.setPower(Math.abs(FRdefaultForce + power));
            backRightMotor.setPower(Math.abs(BRdefaultForce + power));

            phase = "turning right";
        }



        telemetry.addData("Normal Calculation", Math.abs(Math.abs(wantedOrientation) - Math.abs(currentOrientation)));
        //telemetry.addData("on 360 base", -currentOrientation + 180);
        telemetry.addData("Current Orientation", currentOrientation);
        telemetry.addData("Wanted Orientation", wantedOrientation);

    }

    public static void strafeAssist(double currentOrientation, double wantedOrientation, double defaultForce, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, DcMotor backLeftMotor, Telemetry telemetry)
    {
        strafeAssist(currentOrientation, wantedOrientation, defaultForce, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry, true,true,true,true);
    }

    public static void strafeAssist(double currentOrientation, double wantedOrientation, double defaultForce, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, DcMotor backLeftMotor, Telemetry telemetry, boolean usingFrontRight, boolean usingFrontLeft, boolean usingBackRight, boolean usingBackLeft)
    {
        strafeAssist(currentOrientation, wantedOrientation, defaultForce, defaultForce, defaultForce, defaultForce, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry, usingFrontRight, usingFrontLeft, usingBackRight, usingBackLeft);
    }

    public static void strafeAssist(double currentOrientation, double wantedOrientation, double FLdefaultForce, double FRdefaultForce, double BLdefaultForce, double BRdefaultForce, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, DcMotor backLeftMotor, Telemetry telemetry, boolean usingFrontRight, boolean usingFrontLeft, boolean usingBackRight, boolean usingBackLeft) {

        telemetry.addData("runningAutoOrient", "");

        List<DcMotor> motors;

        motors = new ArrayList<>();

        motors.add(frontLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backLeftMotor);
        motors.add(backRightMotor);




        motors.forEach(motor -> {
            if(motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        });


        double wantedDegrees = wantedOrientation + 180;
        double currentDegrees = currentOrientation + 180;

        double distance = Math.abs(currentDegrees - wantedDegrees);
        double distanceMagnitude = distance / 3.6;

        double power = 1 * distanceMagnitude;

        if(power > 1) power = 1;

        telemetry.addData("Distance from target orientation", distance);
        telemetry.addData("Distance Magnitude", distanceMagnitude);
        telemetry.addData("power", power);
        telemetry.update();

        FLdefaultForce = -FLdefaultForce;

        if (currentOrientation < wantedOrientation)
        {
            frontLeftMotor.setPower(FLdefaultForce + power);
            backLeftMotor.setPower(BLdefaultForce + power);

            frontRightMotor.setPower(FRdefaultForce + power);
            backRightMotor.setPower(BRdefaultForce + power);
            telemetry.addData("turning Left", currentOrientation);
            phase = "turning left";

        }

        else if(5d > Math.abs(Math.abs(wantedOrientation) - Math.abs(currentOrientation)))
        {
            telemetry.addData("normal", currentOrientation);
            if(usingFrontLeft) frontLeftMotor.setPower(FLdefaultForce);


            if(usingBackLeft) backLeftMotor.setPower(BLdefaultForce);


            if(usingFrontRight) frontRightMotor.setPower(FRdefaultForce);

            if(usingBackRight) backRightMotor.setPower(BRdefaultForce);

            phase = "normal";

        }
        else if (currentOrientation > wantedOrientation) {
            telemetry.addData("turning Right", currentOrientation);

            frontLeftMotor.setPower(-Math.abs(FLdefaultForce + power));
            backLeftMotor.setPower(-Math.abs(BLdefaultForce + power));

            frontRightMotor.setPower(Math.abs(FRdefaultForce + power));
            backRightMotor.setPower(Math.abs(BRdefaultForce + power));

            phase = "turning right";
        }



        telemetry.addData("Normal Calculation", Math.abs(Math.abs(wantedOrientation) - Math.abs(currentOrientation)));
        //telemetry.addData("on 360 base", -currentOrientation + 180);
        telemetry.addData("Current Orientation", currentOrientation);
        telemetry.addData("Wanted Orientation", wantedOrientation);

    }




}
