package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

@TeleOp
public class encoderValues extends LinearOpMode {



    public void runOpMode()
    {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftWheel");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftWheel");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightWheel");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightWheel");


        DcMotor leftSlideMotor = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlideMotor = hardwareMap.dcMotor.get("rightSlide");

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();

        motors.add(frontLeftMotor);
        motors.add(backLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backRightMotor);
        motors.add(leftSlideMotor);
        motors.add(rightSlideMotor);

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

            telemetry.update();

        }
    }


}
