package org.firstinspires.ftc.teamcode.config.vision.c270;

import android.icu.number.Scale;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorTuningPipeline extends OpenCvPipeline {

    public static Scalar LOWER_BOUND = new Scalar(40, 50, 50);
    public static Scalar UPPER_BOUND = new Scalar(100, 255, 255);

    private Mat hsvMat = new Mat();
    private Mat mask = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat,Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, LOWER_BOUND, UPPER_BOUND, mask);
        return mask;
    }
}
