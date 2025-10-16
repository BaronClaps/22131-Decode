
package org.firstinspires.ftc.teamcode.config.vision;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.vision.opencv.BlobCamera;
import org.firstinspires.ftc.teamcode.config.vision.opencv.Circle;
import org.firstinspires.ftc.teamcode.config.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.teamcode.config.vision.opencv.ColorRange;
import org.firstinspires.ftc.teamcode.config.vision.opencv.ImageRegion;

import java.util.List;

@Config
@TeleOp(name = "Vision Dog", group = "Concept")
public class ArtifactFetcher extends OpMode {
    public ColorBlobLocatorProcessor purpleLocator, greenLocator;
    PIDFController rotController, yController, xController;
    Follower f;
    public static double Pr = 0.003;
    public static double Dr = 0.0003;
    public static double Py = 0.002;
    public static double Dy = 0.0002;
    public static double Px = 0.002;
    public static double Dx = 0.0002;
    public static double Er, Ey, Ex;
    public static double camCenter = 160;
    public static double blobRadGoal = 200;

    @Override
    public void init() {
        f = Constants.createFollower(hardwareMap);
        purpleLocator = new ColorBlobLocatorProcessor.Builder() // creating a new PURPLE color blob locator. this is an sdk example!
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame()) // use the entire camera frame
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
        greenLocator = new ColorBlobLocatorProcessor.Builder() // creating a new GREEN color blob locator. this is an sdk example!
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame()) // use the entire camera frame
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        BlobCamera portal = new BlobCamera.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(greenLocator, purpleLocator)
                .setCameraResolution(new Size(320, 240)) // setting the resolution of the camera. lower resolution = faster looptimes
                .setStreamFormat(BlobCamera.StreamFormat.MJPEG) // have to use MJPEG for the OV camera
                .enableLiveView(true)
               // .setLiveViewContainerId(1011)
                .build();

        telemetry.setMsTransmissionInterval(100);

        rotController = new PIDFController(new PIDFCoefficients(Pr, 0, Dr, 0));
        xController = new PIDFController(new PIDFCoefficients(Px, 0, Dx, 0));
        yController = new PIDFController(new PIDFCoefficients(Py, 0, Dy, 0));
        f.setStartingPose(new Pose());
    }

    @Override
    public void start() {
        f.update();
        f.startTeleopDrive();
    }

    @Override
    public void loop() {
        rotController.setP(Pr);
        rotController.setD(Dr);
        yController.setP(Py);
        yController.setD(Dr);
        xController.setP(Px);
        xController.setD(Dx);
        f.update();

        // creates a new ArrayList of Blobs. I want to use both purple blobs and green blobs
        List<ColorBlobLocatorProcessor.Blob> blobs = purpleLocator.getBlobs();
        blobs.addAll(greenLocator.getBlobs());

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, // contour area is the size the blob takes
                100, 20000, blobs);  // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, // circularity is how circular the blob is
                0.5, 1, blobs);     // filter out non-circular blobs.

        ColorBlobLocatorProcessor.Util.sortByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs
        );

        if (!blobs.isEmpty()) {
            Er = camCenter - blobs.get(0).getCircle().getX();
            Ey = blobRadGoal - blobs.get(0).getCircle().getRadius();
            Ex = camCenter - blobs.get(0).getCircle().getX();

            rotController.updateError(Er);
            yController.updateError(Ey);
            xController.updateError(Ex);

            telemetry.addLine("Er   | Ey   | Ex  ");
            telemetry.addLine(String.format("%.3f, %.3f, %.3f", Er, Ey, Ex));
            telemetry.addLine();

            // drive is my robot centric drive. i feed it my y, x, and rot, and it moves the robot!
            // it works the same way as moving ur joystick

            if (gamepad1.a) {
                f.setTeleOpDrive(
                        yController.run(),
                        xController.run(),
                        rotController.run(),
                        true
                );
            } else
                f.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        } else {
            telemetry.addLine("Er   | Ey   | Ex  ");
            telemetry.addLine(String.format("%.3f, %.3f, %.3f", 0.0, 0.0, 0.0));
            telemetry.addLine();

            f.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            rotController.updateError(0);
            yController.updateError(0);
            xController.updateError(0);
        }

        telemetry.addLine("Circularity | Radius | Center");
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            Circle circleFit = b.getCircle();
            telemetry.addLine(String.format("%5.3f   |   %3d    |  (%3d,%3d)",
                    b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
        }

        telemetry.update();
    }
}
