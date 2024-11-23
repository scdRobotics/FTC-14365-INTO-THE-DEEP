package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.HoughLines;
import static org.opencv.imgproc.Imgproc.boundingRect;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueVisionPipeline extends OpenCvPipeline {

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
    static Mat ycbcrThresh = new Mat();
    Mat ycbcrMorph = new Mat();
    Mat ycbcrErode = new Mat();
    Mat ycbcrEdge = new Mat();
    Mat dst = new Mat();


    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(30, 30));

    //int cameraMonitorViewId = sampleDrive.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", sampleDrive.hardwareMap.appContext.getPackageName());
    //OpenCvCamera openCvCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

    //VisionPipeline visionPipeline = new VisionPipeline();

    public static Rect boundingRect;

    Scalar yellowLowThresh = new Scalar(157, 130, 0);
    Scalar yellowHighThresh = new Scalar(255, 170, 108);

    Scalar blueLowThresh = new Scalar(0, 40, 150);
    Scalar blueHighThresh = new Scalar(90, 160, 290);

    Scalar redLowThresh = new Scalar(10, 170, 20);
    Scalar redHighThresh = new Scalar(100, 300, 140);

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
    public static int getRectX(){
        if(boundingRect != null) {
            return boundingRect.x;
        }
        else return 0;
    }
    @Override
    public Mat processFrame(Mat inputMat) {
        //telemetry.addData("RRRRRRRRRUUUUUNNNNNIIIIIIINNNNNNNNNNNNGGGGGGG", "WE'RE GOING INSANE THIS IS OUR CALL FOR HELP IF YOU SEE THIS IT'S ALREADY TOO LATE PLS HAAAALP");
        //telemetry.update();
        Imgproc.cvtColor(inputMat, ycbcrMat, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(ycbcrMat, ycbcrMat, kernel);
        Core.inRange(ycbcrMat, blueLowThresh, blueHighThresh, ycbcrThresh);
        Imgproc.Canny(ycbcrMorph, ycbcrEdge, 100, 1000, 5, true);

        boundingRect = boundingRect(ycbcrThresh);

        Imgproc.rectangle(ycbcrThresh, boundingRect, new Scalar(255,0,0));

        return ycbcrThresh;
/*
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
        */


    }
}
