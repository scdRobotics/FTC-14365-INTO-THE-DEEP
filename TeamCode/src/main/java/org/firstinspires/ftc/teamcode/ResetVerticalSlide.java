package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ResetVerticalSlide extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor leftSlideMotor = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlideMotor = hardwareMap.dcMotor.get("rightSlide");

        waitForStart();

        while(opModeIsActive())
        {
            boolean dPadUp = gamepad1.dpad_up;;
            boolean dPadDown = gamepad1.dpad_down;

            boolean aButtonDown = gamepad1.a;

            if(dPadUp)
            {
                leftSlideMotor.setPower(-0.8);
                rightSlideMotor.setPower(0.8);
            }
            if(dPadDown)
            {

                leftSlideMotor.setPower(0.8);
                rightSlideMotor.setPower(-0.8);
            }
            if(!dPadUp && !dPadDown)
            {
                leftSlideMotor.setPower(0);
                rightSlideMotor.setPower(0);
            }

            telemetry.addData("leftSlideMotor pos", leftSlideMotor.getCurrentPosition());
            telemetry.addData("rightSlideMotor pos", rightSlideMotor.getCurrentPosition());

            telemetry.update();

            if(aButtonDown)
            {
                leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }
}
