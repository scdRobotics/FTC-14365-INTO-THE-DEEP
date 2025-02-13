package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

@TeleOp
public class encoderValues extends LinearOpMode {



    public void runOpMode()
    {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");


        DcMotor leftSlideMotor = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlideMotor = hardwareMap.dcMotor.get("rightSlide");

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();

        motors.add(frontLeftMotor);
        motors.add(backLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backRightMotor);
        motors.add(leftSlideMotor);
        motors.add(rightSlideMotor);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);


        waitForStart();

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        while(opModeIsActive())
        {
            telemetry.addData("frontLeftMotor pos", frontLeftMotor.getCurrentPosition());
            telemetry.addData("frontRightMotor pos", frontRightMotor.getCurrentPosition());
            telemetry.addData("backLeftMotor pos", backLeftMotor.getCurrentPosition());
            telemetry.addData("backRightMotor pos", backRightMotor.getCurrentPosition());

            telemetry.addData("rightSlide pos", rightSlideMotor.getCurrentPosition());
            telemetry.addData("leftSlide pos", leftSlideMotor.getCurrentPosition());

            telemetry.addData("robot heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            telemetry.update();

        }
    }


}
