package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class ServoTestAuto extends LinearOpMode {


    /// 0.1 for low max, 0.7 for high max
    @Override

    public void runOpMode() throws InterruptedException {

        Servo bucketRotateServo = hardwareMap.servo.get("bucketServo");

        telemetry.addData("Current Position: ", bucketRotateServo.getPosition());
        telemetry.addData("Current Direction: ", bucketRotateServo.getDirection());
        telemetry.update();

        waitForStart();

        telemetry.addData("0 Position", "");
        telemetry.update();
        bucketRotateServo.setPosition(0);
        sleep(1000);

        telemetry.addData("0.1 Position", "");
        telemetry.update();
        bucketRotateServo.setPosition(0.1);
        sleep(1000);

        telemetry.addData("0.2 Position", "");
        telemetry.update();
        bucketRotateServo.setPosition(0.2);
        sleep(1000);

        telemetry.addData("0.3 Position", "");
        telemetry.update();
        bucketRotateServo.setPosition(0.3);
        sleep(1000);

        telemetry.addData("0.4 Position", "");
        telemetry.update();
        bucketRotateServo.setPosition(0.4);
        sleep(1000);

        telemetry.addData("0.5 Position", "");
        telemetry.update();
        bucketRotateServo.setPosition(0.5);
        sleep(1000);

        telemetry.addData("0.6 Position", "");
        telemetry.update();
        bucketRotateServo.setPosition(0.6);
        sleep(1000);

        telemetry.addData("0.7 Position", "");
        telemetry.update();
        bucketRotateServo.setPosition(0.7);
        sleep(1000);


        telemetry.addData("0.8 Position", "");
        telemetry.update();
        bucketRotateServo.setPosition(0.8);
        sleep(1000);

        telemetry.addData("0.9 Position", "");
        telemetry.update();
        bucketRotateServo.setPosition(0.9);
        sleep(1000);

        telemetry.addData("1 Position", "");
        telemetry.update();
        bucketRotateServo.setPosition(1);
        sleep(1000);
    }
}



