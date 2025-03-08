package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;

@TeleOp
public class encoderValues extends LinearOpMode {

    IMU imu;

    public void runOpMode()
    {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");


        DcMotor leftSlideMotor = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlideMotor = hardwareMap.dcMotor.get("rightSlide");

        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        DcMotor par0 = hardwareMap.dcMotor.get("bucketSlideMotor");
        DcMotor par1 = hardwareMap.dcMotor.get("par1");
        DcMotor perp = hardwareMap.dcMotor.get("rightSlide");

        par0.setDirection(DcMotorSimple.Direction.REVERSE);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);
        
        final double trackwidth =  124;
        double odometryHeading;

        double finalHeading;

        double differenceInPar0;
        double differenceInPar1;
        double differenceInPerp;
        double averageDifferenceInPar;
        double differenceInOdoHeading;


        double par0Pos = 0;
        double par1Pos = 0;
        double perpPos = 0;
        double par0LastPos = 0;
        double par1LastPos = 0;
        double perpLastPos = 0;

        Pose2D robotPose;


        par0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        par1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();

        motors.add(frontLeftMotor);
        motors.add(backLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backRightMotor);
        motors.add(leftSlideMotor);
        motors.add(rightSlideMotor);

        int i = 0;

        waitForStart();

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        robotPose = new Pose2D(INCH, 0, 0, DEGREES, 0);
        while(opModeIsActive())
        {
            par0Pos = par0.getCurrentPosition();
            par1Pos = par1.getCurrentPosition();
            perpPos = perp.getCurrentPosition();



            odometryHeading = (par0Pos - par1Pos) / trackwidth;

            differenceInPar0 = par0Pos - par0LastPos;
            differenceInPar1 = par1Pos - par1LastPos;

            averageDifferenceInPar = (differenceInPar0 - differenceInPar1) * 2;

            differenceInOdoHeading = (differenceInPar0 - differenceInPar1) / trackwidth;

            differenceInPerp = perpPos - perpLastPos;

            finalHeading = -(odometryHeading - imu.getRobotYawPitchRollAngles().getYaw(DEGREES) * 2);

            robotPose = new Pose2D(INCH, robotPose.getX(INCH) + (differenceInPar0 * differenceInPar1 < 0 ? 0 : averageDifferenceInPar), robotPose.getY(INCH) + (differenceInPar0 * differenceInPar1 < 0 ? 0 : differenceInPerp), DEGREES, robotPose.getHeading(DEGREES) + differenceInOdoHeading);

            par0LastPos = par0Pos;
            par1LastPos = par1Pos;
            perpLastPos = perpPos;

            telemetry.addData("frontLeftMotor pos", frontLeftMotor.getCurrentPosition());
            telemetry.addData("frontRightMotor pos", frontRightMotor.getCurrentPosition());
            telemetry.addData("backLeftMotor pos", backLeftMotor.getCurrentPosition());
            telemetry.addData("backRightMotor pos", backRightMotor.getCurrentPosition());

            telemetry.addData("rightSlide pos", rightSlideMotor.getCurrentPosition());
            telemetry.addData("leftSlide pos", leftSlideMotor.getCurrentPosition());

            telemetry.addData("distanceSensor", distanceSensor.getDistance(INCH));

            telemetry.addData("robot heading", imu.getRobotYawPitchRollAngles().getYaw(DEGREES));

            telemetry.addData("par0 pos", par0.getCurrentPosition());
            telemetry.addData("par1 pos", par1.getCurrentPosition());
            telemetry.addData("perp pos", perp.getCurrentPosition());

            telemetry.addData("par0 position change" , differenceInPar0);
            telemetry.addData("par1 position change", differenceInPar1);
            telemetry.addData("perp position change" , differenceInPerp);

            telemetry.addData("OdometryHeading calculation", odometryHeading);
            telemetry.addData("change in Odometry heading", differenceInOdoHeading);

            telemetry.addData("final heading calculation", finalHeading);


            telemetry.addData("RobotPose x val", robotPose.getX(INCH));
            telemetry.addData("RobotPose y val", robotPose.getY(INCH));
            telemetry.addData("RobotPose Heading", robotPose.getHeading(DEGREES));

            telemetry.addData("turning = ", differenceInPar0 * differenceInPar1 < 0);

            telemetry.update();



        }
    }


}
