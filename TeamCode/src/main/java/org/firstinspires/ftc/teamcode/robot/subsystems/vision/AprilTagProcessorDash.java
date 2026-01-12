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

    private Integer onScreenWidth = null;
    private Integer onScreenHeight = null;
    private float scaleBmpPxToCanvasPx = 1.0f;
    private float scaleCanvasDensity = 1.0f;

    public AprilTagProcessorDash(AprilTagProcessor processor) {
        this.processor = processor;
        this.onScreenWidth = null;
        this.onScreenHeight = null;
        this.scaleBmpPxToCanvasPx = 1.0f;
        this.scaleCanvasDensity = 1.0f;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        processor.init(width, height, calibration);
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // we gnore the ftc canvas
        // create Canvas backed by THIS bitmap
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        Object result = processor.processFrame(frame, captureTimeNanos);

//        // i think this might be needed to get a mutable bitmap from the mat, but it is very expensive so id
//        Bitmap mutableBitmap = b.copy(Bitmap.Config.RGB_565, true);
        Canvas canvas = new Canvas(b);

        processor.onDrawFrame(
                canvas,
                onScreenWidth != null ? onScreenWidth : frame.width(),
                onScreenHeight != null ? onScreenHeight : frame.height(),
                scaleBmpPxToCanvasPx,
                scaleCanvasDensity,
                result
        );

        lastFrame.set(b);

        return result;
    }

    @Override
    public void onDrawFrame(Canvas canvasSkb, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        this.onScreenWidth = onscreenWidth;
        this.onScreenHeight = onscreenHeight;
        this.scaleBmpPxToCanvasPx = scaleBmpPxToCanvasPx;
        this.scaleCanvasDensity = scaleCanvasDensity;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public ArrayList<AprilTagDetection> getDetections() {
        return processor.getDetections();
    }
}
