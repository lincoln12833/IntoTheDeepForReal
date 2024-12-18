package org.firstinspires.ftc.teamcode.MM.MM;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class MM_VisionPortal {
    public MM_OpMode opMode;


    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;


    public MM_VisionPortal(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public boolean setPosFromApriltag(){
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if(!detections.isEmpty()){
            MM_Robot.robotX = detections.get(0).robotPose.getPosition().x;
            MM_Robot.robotY = detections.get(0).robotPose.getPosition().y;
            MM_Robot.position = new Pose2D(DistanceUnit.INCH, MM_Robot.robotX, MM_Robot.robotY, AngleUnit.DEGREES, detections.get(0).robotPose.getOrientation().getYaw()); //odometryPos.getHeading(AngleUnit.DEGREES));

            opMode.multipleTelemetry.addData("got in if?", "yes");
            opMode.multipleTelemetry.addData("robot pos x", detections.get(0).robotPose.getPosition().x);
            opMode.multipleTelemetry.addData("robot pos y", detections.get(0).robotPose.getPosition().y);
            return true;
        }

        return false;
    }

    private void init(){
        final CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();

        aprilTagProcessor =  new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .addProcessor(cameraStreamProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor, 0);
    }



    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
}
