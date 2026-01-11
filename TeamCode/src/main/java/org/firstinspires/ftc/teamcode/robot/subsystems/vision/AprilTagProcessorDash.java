package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class AprilTagProcessorDash implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public final AprilTagProcessor processor;

    public AprilTagProcessorDash(AprilTagProcessor processor) {
        this.processor = processor;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return processor.processFrame(frame, captureTimeNanos);
    }

    @Override
    public void onDrawFrame(Canvas canvasSkb, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // we gnore the ftc canvas
        Bitmap bitmap = Bitmap.createBitmap(
                frame.width(),
                frame.height(),
                Bitmap.Config.RGB_565
        );
        Utils.matToBitmap(frame, bitmap);

        // create Canvas backed by THIS bitmap
        Canvas canvas = new Canvas(bitmap);

        processor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public ArrayList<AprilTagDetection> getDetections() {
        return processor.getDetections();
    }
}
