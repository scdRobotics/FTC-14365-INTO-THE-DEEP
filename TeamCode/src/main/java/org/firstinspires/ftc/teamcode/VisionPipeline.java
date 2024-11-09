package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.THRESH_BINARY;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY_INV;
import static org.opencv.imgproc.Imgproc.boundingRect;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.RotatedRect;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint2f;

public class VisionPipeline extends OpenCvPipeline {

    enum Stage {
        ycbcr,
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

    public static Rect boundingRect;

    Scalar lowThresh = new Scalar(16, 16, 16);
    Scalar highThresh = new Scalar(235,240, 240);

    Scalar red = new Scalar(255,0,0);

    private VisionColorDetection.Stage stageToRenderToViewport = VisionColorDetection.Stage.DST;
    private VisionColorDetection.Stage[] stages = VisionColorDetection.Stage.values();

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

    public Mat processFrame(Mat inputMat) {

        Imgproc.cvtColor(inputMat, ycbcrMat, Imgproc.COLOR_RGB2HLS);

        Core.inRange(ycbcrMat, lowThresh, highThresh, ycbcrThresh);


        boundingRect = boundingRect(ycbcrThresh);


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
