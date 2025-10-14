package org.firstinspires.ftc.teamcode.config.vision.c270;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class WatershedProc extends OpenCvPipeline {

    public enum DebugView {
        COLOR_MASK,
        NOISE_REDUCTION,
        ARTIFACT_CORES,
        INPUT_WATERSHED,
        WATERSHED,
        FINAL
    }

    public static DebugView VIEW_MODE = DebugView.FINAL;

    public static final Scalar LOWER_GREEN = new Scalar(75, 65, 65);
    public static final Scalar UPPER_GREEN = new Scalar(100, 255, 255);

    private volatile int numObjectsFound = 0;
    private List<MatOfPoint> finalContours = new ArrayList<>();

    private static final double SEPARATION_THRESHOLD = 0.7;
    private static final int MORPH_OPEN_ITERATIONS = 10;
    private static final double POSITION_THRESHOLD = 50.0; // Max distance to match same artifact (pixels)

    private Mat hsvMat = new Mat();
    private Mat mask = new Mat();
    private Mat opening = new Mat();
    private Mat distTransform = new Mat();
    private Mat sureFg = new Mat();
    private Mat markers = new Mat();
    private Mat unknown = new Mat();
    private Mat sureBg = new Mat();
    private Mat componentMask = new Mat();
    private Mat sureFgEroded = new Mat();
    private Mat inputCopy = new Mat();
    boolean hasLooped = true;
    private Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

    public volatile List<TrackedArtifact> detectedArtifacts = new ArrayList<>();
    private int nextArtifactId = 0;

    public static class TrackedArtifact {
        public int id;
        public Point position;
        public double area;
        public double width;
        public double height;
        public int framesTracked;

        public TrackedArtifact(int id, Point position, double area, double width, double height) {
            this.id = id;
            this.position = position;
            this.area = area;
            this.width = width;
            this.height = height;
            this.framesTracked = 1;
        }
    }

    public boolean changeViewMode() {
        switch (VIEW_MODE) {
            case COLOR_MASK:
                VIEW_MODE = DebugView.NOISE_REDUCTION;
                return true;
            case NOISE_REDUCTION:
                VIEW_MODE = DebugView.WATERSHED;
                return true;
            case WATERSHED:
                VIEW_MODE = DebugView.ARTIFACT_CORES;
                return true;
            case FINAL:
                VIEW_MODE = DebugView.COLOR_MASK;
                return true;
            case ARTIFACT_CORES:
            default:
                VIEW_MODE = DebugView.FINAL;
                return true;
        }
    }

    public DebugView getViewMode() {
        return VIEW_MODE;
    }

    private double distanceBetween(Point p1, Point p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private TrackedArtifact matchArtifactToExisting(Point newPosition, double area, double width, double height) {
        TrackedArtifact closest = null;
        double closestDistance = POSITION_THRESHOLD;

        for (TrackedArtifact tracked : detectedArtifacts) {
            double distance = distanceBetween(newPosition, tracked.position);
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = tracked;
            }
        }

        if (closest != null) {
            closest.position = newPosition;
            closest.area = area;
            closest.width = width;
            closest.height = height;
            closest.framesTracked++;
            return closest;
        }

        return null;
    }

    @Override
    public Mat processFrame(Mat input) {
        hasLooped = false;
        input.copyTo(inputCopy);

        try {
            Imgproc.cvtColor(inputCopy, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, LOWER_GREEN, UPPER_GREEN, mask);

            Imgproc.morphologyEx(mask, opening, Imgproc.MORPH_OPEN, kernel, new Point(-1, -1), MORPH_OPEN_ITERATIONS);
            Imgproc.GaussianBlur(opening, opening, new Size(5, 5), 0);
            Imgproc.distanceTransform(opening, distTransform, Imgproc.DIST_L2, 3);
            Imgproc.erode(opening, sureFgEroded, kernel, new Point(-1, -1), 2);
            Imgproc.distanceTransform(opening, distTransform, Imgproc.DIST_L2, 5);
            Core.normalize(distTransform, distTransform, 0, 1, Core.NORM_MINMAX);
            Imgproc.threshold(distTransform, sureFg, SEPARATION_THRESHOLD, 1, Imgproc.THRESH_BINARY);
            sureFg.convertTo(sureFg, CvType.CV_8U, 255);
            Imgproc.connectedComponents(sureFg, markers);

            Imgproc.dilate(opening, sureBg, kernel, new Point(-1, -1), 3);
            Core.subtract(sureBg, sureFg, unknown);
            Core.add(markers, new Scalar(1), markers);
            markers.setTo(new Scalar(0), unknown);

            if (inputCopy.channels() == 1) {
                Imgproc.cvtColor(inputCopy, inputCopy, Imgproc.COLOR_GRAY2RGB);
            } else if (inputCopy.channels() == 4) {
                Imgproc.cvtColor(inputCopy, inputCopy, Imgproc.COLOR_RGBA2RGB);
            } else if (inputCopy.channels() == 3 && inputCopy.depth() != CvType.CV_8U) {
                inputCopy.convertTo(inputCopy, CvType.CV_8UC3, 255.0);
            }

            if (inputCopy != null && !inputCopy.empty() &&
                    markers != null && !markers.empty() &&
                    inputCopy.size().equals(markers.size()) &&
                    inputCopy.type() == CvType.CV_8UC3 &&
                    markers.type() == CvType.CV_32SC1) {
                Imgproc.watershed(inputCopy, markers);
            }

            List<TrackedArtifact> newDetections = new ArrayList<>();
            int numObjects = (int) Core.minMaxLoc(markers).maxVal;

            for (int i = 2; i <= numObjects; i++) {
                componentMask = new Mat(markers.size(), CvType.CV_8UC1);
                Core.compare(markers, new Scalar(i), componentMask, Core.CMP_EQ);

                List<MatOfPoint> contours = new ArrayList<>();
                Imgproc.findContours(componentMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                if (!contours.isEmpty()) {
                    MatOfPoint contour = contours.get(0);
                    double area = Imgproc.contourArea(contour);
                    RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                    TrackedArtifact matched = matchArtifactToExisting(rotatedRect.center, area, rotatedRect.size.width, rotatedRect.size.height);
                    if (matched == null) {
                        matched = new TrackedArtifact(nextArtifactId++, rotatedRect.center, area, rotatedRect.size.width, rotatedRect.size.height);
                    }
                    newDetections.add(matched);

                    Point[] vertices = new Point[4];
                    rotatedRect.points(vertices);
                    for (int j = 0; j < 4; j++) {
                        Imgproc.line(inputCopy, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
                    }
                    Imgproc.putText(inputCopy, String.format("ID:%d W:%.0f H:%.0f", matched.id, rotatedRect.size.width, rotatedRect.size.height), rotatedRect.center, Imgproc.FONT_ITALIC, 0.5, new Scalar(255, 255, 255), 1);
                }
            }

            this.detectedArtifacts = newDetections;
            Imgproc.putText(inputCopy, "Found: " + detectedArtifacts.size(), new Point(10, 30), Imgproc.FONT_ITALIC, 0.7, new Scalar(255, 255, 255), 2);

            switch (VIEW_MODE) {
                case COLOR_MASK:
                    return mask;
                case NOISE_REDUCTION:
                    return opening;
                case ARTIFACT_CORES:
                    sureFg.convertTo(sureFg, CvType.CV_8U, 255);
                    return sureFg;
                case WATERSHED:
                    markers.convertTo(markers, CvType.CV_8U, 10);
                    return markers;
                case FINAL:
                default:
                    return inputCopy;
            }
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
    }

    public int getNumObjectsFound() {
        return detectedArtifacts.size();
    }
}