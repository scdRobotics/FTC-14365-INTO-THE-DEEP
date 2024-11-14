package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY_INV;
import static org.opencv.imgproc.Imgproc.boundingRect;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.RotatedRect;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint2f;

public class VisionPipeline extends OpenCvPipeline {

    enum Stage {
        YCbCr,
        THRESH,
        MORPH,
        ERODE,
        EDGE,
        DST,
        RAW_IMAGE
    }

    Mat ycbcrMat = new Mat();
    Mat ycbcrThresh = new Mat();
    Mat ycbcrMorph = new Mat();
    Mat ycbcrErode = new Mat();
    Mat ycbcrEdge = new Mat();
    Mat dst = new Mat();

    //int cameraMonitorViewId = sampleDrive.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", sampleDrive.hardwareMap.appContext.getPackageName());
    //OpenCvCamera openCvCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

    //VisionPipeline visionPipeline = new VisionPipeline();

    public static Rect boundingRect;

    Scalar lowThresh = new Scalar(0, 130, 0);
    Scalar highThresh = new Scalar(255, 170, 108);

    Scalar red = new Scalar(255,0,0);

    private Stage stageToRenderToViewport = Stage.DST;
    private Stage[] stages = Stage.values();

    @Override
    public void onViewportTapped() {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    public static Rect getRect()
    {
        return boundingRect;
    }
    static int i = 0;
    public static int getI() {return i;}

    public Mat processFrame(Mat inputMat) {
        telemetry.addData("RRRRRRRRRUUUUUNNNNNIIIIIIINNNNNNNNNNNNGGGGGGG", "WE'RE GOING INSANE THIS IS OUR CALL FOR HELP IF YOU SEE THIS IT'S ALREADY TOO LATE PLS HAAAALP");
        telemetry.update();
        Imgproc.cvtColor(inputMat, ycbcrMat, Imgproc.COLOR_RGB2YCrCb);
        i++;
        Core.inRange(ycbcrMat, lowThresh, highThresh, ycbcrThresh);


        boundingRect = boundingRect(ycbcrThresh);
        Imgproc.rectangle(inputMat, boundingRect, new Scalar(255,0,0));

        switch (stageToRenderToViewport) {
            case YCbCr: {
                return ycbcrMat;
            }

            case THRESH: {
                return this.ycbcrThresh;
            }

            case MORPH: {
                return ycbcrMorph;
            }
            case ERODE: {
                return ycbcrErode;
            }

            case EDGE: {
                return ycbcrEdge;
            }

            case RAW_IMAGE: {
                return inputMat;
            }

            case DST: {
                return dst;
            }

            default: {
                return inputMat;
            }
        }


    }
}
