package org.firstinspires.ftc.teamcode.config.vision.c270;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Disabled
@TeleOp
public class CustomCamera extends OpMode {
    WatershedProc watershedProc;
    OpenCvWebcam camera;

    @Override
    public void init() {
        watershedProc = new WatershedProc();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDeviceAsync(
            new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(1280, 720, OpenCvCameraRotation.SENSOR_NATIVE, OpenCvWebcam.StreamFormat.MJPEG);
                }

                @Override
                public void onError(int errorCode) {
                }
            }
        );

        telemetry.addLine("loaded!");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed())
            watershedProc.changeViewMode();

        telemetry.addData("view mode", watershedProc.getViewMode());
        telemetry.addData("num obj found", watershedProc.getNumObjectsFound());
        telemetry.update();

    }
}