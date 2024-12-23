package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ResetBucketSlide extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor rightBucketMotor = hardwareMap.dcMotor.get("rightBucketSlideMotor");
        DcMotor leftBucketMotor = hardwareMap.dcMotor.get("leftBucketSlideMotor");

        waitForStart();

        while(opModeIsActive())
        {
            boolean Gp2YButtonDown = gamepad2.y;
            boolean Gp2AButtonDown = gamepad2.a;
            boolean Gp2BButtonDown = gamepad2.b;

            if(Gp2YButtonDown)
            {
                rightBucketMotor.setTargetPosition(rightBucketMotor.getCurrentPosition() + 100);
                rightBucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightBucketMotor.setPower(0.3);
                leftBucketMotor.setTargetPosition(leftBucketMotor.getCurrentPosition() + 100);
                leftBucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftBucketMotor.setPower(0.5);
            }
            if(Gp2AButtonDown)
            {
                rightBucketMotor.setTargetPosition(rightBucketMotor.getCurrentPosition() - 100);
                rightBucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightBucketMotor.setPower(-0.3);
                leftBucketMotor.setTargetPosition(leftBucketMotor.getCurrentPosition() - 80);
                leftBucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftBucketMotor.setPower(-0.5);
            }


            if(Gp2BButtonDown){
                rightBucketMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBucketMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }
}
