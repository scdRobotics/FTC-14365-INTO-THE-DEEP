package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class linearSlidePidLoop {

    public Void linearSlideController(DcMotor baseSlide, DcMotor secondSlide,Telemetry telemetry)
    {
        double baseSlidePos = baseSlide.getCurrentPosition();
        double secondSlidePos = secondSlide.getCurrentPosition();

        if(Math.abs(secondSlidePos) - Math.abs(baseSlidePos) > -50)
        {
            secondSlide.setTargetPosition(secondSlide.getCurrentPosition() + 10);
            secondSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            secondSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            secondSlide.setPower(0.2d);
            telemetry.addData("Slide moving down", "");
        }
        else if(50 >= Math.abs(Math.abs(secondSlidePos) - Math.abs(baseSlidePos)))
        {
            secondSlide.setTargetPosition(secondSlide.getCurrentPosition());
            secondSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            secondSlide.setPower(0);
            telemetry.addData("Slide done moving", "");
        }
        if(Math.abs(secondSlidePos) - Math.abs(baseSlidePos) > 50)
        {
            secondSlide.setTargetPosition(secondSlide.getCurrentPosition() + 10);
            secondSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            secondSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            secondSlide.setPower(0.2d);

            telemetry.addData("Slide moving up", "");
        }
        telemetry.addData("Running PID loop", Math.abs(Math.abs(secondSlidePos) - Math.abs(baseSlidePos)));
        return null;
    }
}
