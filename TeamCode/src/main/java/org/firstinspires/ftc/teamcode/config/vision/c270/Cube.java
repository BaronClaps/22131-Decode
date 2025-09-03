package org.firstinspires.ftc.teamcode.config.vision.c270;

import android.graphics.Canvas;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Cube implements VisionProcessor {

    public Scalar lower = new Scalar(87, 41, 170);
    public Scalar upper = new Scalar(202.6, 174.3, 202.6);

    private Mat labMat = new Mat();
    private Mat binaryMat = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(android.graphics.Color.GREEN);
        paint.setStrokeWidth(2 * scaleCanvasDensity);
        paint.setStyle(Paint.Style.STROKE);

        for (MatOfPoint contour : contours) {
            Point[] points = contour.toArray();
            for (int i = 0; i < points.length; i++) {
                Point p1 = points[i];
                Point p2 = points[(i + 1) % points.length];

                canvas.drawLine(
                        (float) (p1.x * scaleBmpPxToCanvasPx),
                        (float) (p1.y * scaleBmpPxToCanvasPx),
                        (float) (p2.x * scaleBmpPxToCanvasPx),
                        (float) (p2.y * scaleBmpPxToCanvasPx),
                        paint
                );
            }
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, labMat, Imgproc.COLOR_RGB2Lab);
        Core.inRange(labMat, lower, upper, binaryMat);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(binaryMat, binaryMat, Imgproc.MORPH_CLOSE, kernel);

        contours.clear(); // Clear previous frame's contours
        Mat hierarchy = new Mat();
        Imgproc.findContours(binaryMat.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        return binaryMat;
    }
}

