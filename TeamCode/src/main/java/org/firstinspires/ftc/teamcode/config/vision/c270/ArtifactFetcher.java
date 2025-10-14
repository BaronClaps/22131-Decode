
package org.firstinspires.ftc.teamcode.config.vision.c270;

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
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.vision.c270.opencv.BlobCamera;
import org.firstinspires.ftc.teamcode.config.vision.c270.opencv.Circle;
import org.firstinspires.ftc.teamcode.config.vision.c270.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.teamcode.config.vision.c270.opencv.ColorRange;
import org.firstinspires.ftc.teamcode.config.vision.c270.opencv.ImageRegion;

import java.util.List;

@Config
@TeleOp(name = "Vision Auto", group = "Concept")
public class ArtifactFetcher extends OpMode {
    public ColorBlobLocatorProcessor purpleLocator, greenLocator;
    PIDFController rotController, yController, xController;
    Follower f;
    public double Pr = 0.0003;
    public double Py = 0.0002;
    public double Px = 0.0002;
    public double camCenter = (double) 1280 /2;
    public double blobRadGoal = 300;
    public double INTAKE_ACTIVATION_RADIUS = 45;

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

        BlobCamera portal = new BlobCamera.Builder() // Building a new vision portal
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // setting the camera
                .addProcessors(greenLocator, purpleLocator) // adding the two processors i just made
                .setCameraResolution(new Size(1280, 720)) // setting the resolution of the camera. lower resolution = faster looptimes
                .setStreamFormat(BlobCamera.StreamFormat.MJPEG) // have to use MJPEG for the OV camera
                .setLiveViewContainerId(0)
                .build();

        telemetry.setMsTransmissionInterval(100);   // Speed up telemetry updates for debugging. (normally its 250ms)

        rotController = new PIDFController(new PIDFCoefficients(Pr, 0,0 ,0));
        xController = new PIDFController(new PIDFCoefficients(Px, 0,0 ,0));
        yController = new PIDFController(new PIDFCoefficients(Py, 0,0 ,0));
        f.setStartingPose(new Pose());
    }

    @Override
    public void start() {
        f.update();
        f.startTeleopDrive();
    }
    @Override
    public void loop() {

            f.update();
            // creates a new ArrayList of Blobs. I want to use both purple blobs and green blobs
            List<ColorBlobLocatorProcessor.Blob> blobs = purpleLocator.getBlobs();
            blobs.addAll(greenLocator.getBlobs());

            // you can filter blobs to remove unwanted data & distractions. these are from the SDK
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, // contour area is the size the blob takes
                    100, 20000, blobs);  // filter out very small blobs.

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, // circularity is how circular the blob is
                    0.5, 1, blobs);     // filter out non-circular blobs.


            // I want to sort the blobs so that the robot focuses on the largest blob first.
            // remember, contour area is the size of the blob
            ColorBlobLocatorProcessor.Util.sortByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,  SortOrder.DESCENDING, blobs
            );

            // telemtry stuff from the sdk
            telemetry.addLine("Circularity Radius Center");
            // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                Circle circleFit = b.getCircle();
                telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
                        b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
            }

            // this is my code for the PIDs!
            if (!blobs.isEmpty()) { // i only want to be moving if there are blobs that i can see.
                // goals are where the PID is going to try to get the value to.
                // the point of the robot strafing/rotating is to get the robot in the center of the camera (in the x direction)

                rotController.updateError(camCenter - blobs.get(0).getCircle().getX());
                yController.updateError(blobRadGoal - blobs.get(0).getCircle().getRadius());
                xController.updateError(camCenter - blobs.get(0).getCircle().getX());

                // drive is my robot centric drive. i feed it my y, x, and rot, and it moves the robot!
                // it works the same way as moving ur joystick

                if (gamepad1.a) {
                    f.setTeleOpDrive(
                            // out of my list of sorted blobs, get the first one (in an array that's 0), get the circle, and get the lateral x.
                            // it's going to try to get the x to be equal to 160 (the center of the camera)
                            rotController.run(),
                            // same thing, but it's trying to change the radius
                            yController.run(),
                            xController.run(),
                            true
                    );
                } else
                    f.setTeleOpDrive(-gamepad1.left_stick_y,-gamepad1.left_stick_x,-gamepad1.right_stick_x, true);
               /* if (blobs.get(0).getCircle().getRadius() > INTAKE_ACTIVATION_RADIUS && intake.getStoredArtifact() == ChoppedIntake.ArtifactColor.NONE) {
                    intake.run(0.7);
                } else {
                    intake.stop();
                } */
            } else {
                // if there are no blobs, my pids shouldnt be trying to do anything and my robot shouldnt drive on its own
                f.setTeleOpDrive(-gamepad1.left_stick_y,-gamepad1.left_stick_x,-gamepad1.right_stick_x, true);
                rotController.updateError(0);
                yController.updateError(0);
                xController.updateError(0);
                //intake.stop();
            }
            /*if (gamepad1.a && intake.getStoredArtifact() != ChoppedIntake.ArtifactColor.NONE) {
                intake.reset();
                telemetry.addData("Intake is Reset","yes");
            }*/
            //telemetry.addData("Artifact", intake.getStoredArtifact());
            //telemetry.addData("Color Sensor val", String.valueOf(intake.alpha), intake.red, intake.blue, intake.green);
            telemetry.update();
        }
    }
