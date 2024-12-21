package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.HoughLines;
import static org.opencv.imgproc.Imgproc.boundingRect;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

public class YellowVisionPipeline extends OpenCvPipeline {

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

    //int cameraMonitorViewId = sampleDrive.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", sampleDrive.hardwareMap.appContext.getPackageName());
    //OpenCvCamera openCvCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

    //VisionPipeline visionPipeline = new VisionPipeline();

    public static Rect boundingRect;

    Scalar yellowLowThresh = new Scalar(130, 130, 40);
    Scalar yellowHighThresh = new Scalar(255, 170, 108);

    Scalar blueLowThresh = new Scalar(0, 40, 150);
    Scalar blueHighThresh = new Scalar(90, 160, 290);

    Scalar redLowThresh = new Scalar(10, 170, 20);
    Scalar redHighThresh = new Scalar(100, 300, 140);

    Scalar red = new Scalar(255,0,0);

    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(30, 30));

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
    public static Mat getycbcrEdge()
    {
        return ycbcrThresh;
    }

    @Override
    public Mat processFrame(Mat inputMat) {
        Imgproc.cvtColor(inputMat, ycbcrMat, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(ycbcrMat, ycbcrMat, kernel);
        Core.inRange(ycbcrMat, yellowLowThresh, yellowHighThresh, ycbcrThresh);
        Mat ycbcrLine = new Mat();
        Imgproc.Canny(ycbcrMorph, ycbcrLine, 0, 1000);

        boundingRect = boundingRect(ycbcrThresh);
        Imgproc.rectangle(ycbcrThresh, boundingRect, new Scalar(255,255,255));

        Mat lineMat = new Mat();
        ycbcrLine.copyTo(lineMat);
        Mat lines = new Mat();
        Imgproc.HoughLines(ycbcrEdge, lines, 1, Math.PI/180, 120, 0, 0, -5*Math.PI/180, 5*Math.PI/180);


        double[] rovioListEdges = new double[lines.rows()];

        for (int x = 0; x < lines.rows(); x++) {
            double theta = lines.get(x, 0)[1];
            double rho = lines.get(x, 0)[0];
            double a = Math.cos(theta), b = Math.sin(theta);
            double x0 = a*rho, y0 = b*rho;
            Point pt1 = new Point(Math.round(x0 + 1000*(-b)), Math.round(y0 + 1000*(a)));
            Point pt2 = new Point(Math.round(x0 - 1000*(-b)), Math.round(y0 - 1000*(a)));
            Imgproc.line(ycbcrEdge, pt1, pt2, new Scalar(0, 255, 0), 3, Imgproc.LINE_AA, 0);
            rovioListEdges[x] = x0;
        }
        Arrays.sort(rovioListEdges);
        return ycbcrEdge;
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
