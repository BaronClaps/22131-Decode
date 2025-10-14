//package org.firstinspires.ftc.teamcode.config.vision.c270;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import android.util.Size;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//import dev.nextftc.ftc.NextFTCOpMode;
//
//@TeleOp
//public class ColorTuningPipelineImpl extends NextFTCOpMode {
//    ColorTuningPipeline colorTune;
//    @Override
//    public void onInit() {
//        colorTune = new ColorTuningPipeline();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        camera.setPipeline(colorTune);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//
//        telemetry.addLine("loaded!");
//    }
//
//    @Override
//    public void onUpdate() {
//        telemetry.update();
//    }
//}