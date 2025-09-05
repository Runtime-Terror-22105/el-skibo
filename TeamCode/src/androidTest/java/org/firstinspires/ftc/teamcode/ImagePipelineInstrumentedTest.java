package org.firstinspires.ftc.teamcode;

import android.content.ContentValues;
import android.content.Context;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Rect;
import android.net.Uri;
import android.os.Environment;
import android.provider.MediaStore;
import android.util.Log;

import androidx.test.platform.app.InstrumentationRegistry;

import junit.framework.TestCase;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.robot.match.MatchColors;
import org.firstinspires.ftc.teamcode.robot.match.SampleColor;
import org.firstinspires.ftc.teamcode.robot.vision.TerrorBlobLocatorProcessor;
import org.firstinspires.ftc.teamcode.robot.vision.TerrorCameraVisionPortal;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.InputStream;
import java.io.OutputStream;

public class ImagePipelineInstrumentedTest extends TestCase {
    private static final String INPUT_FILE = "bumhulgoonma2.png";

    static {
        System.loadLibrary("opencv_java4");
    }

    // NOTE: Run `adb pull /sdcard/Download/<OUTPUT_FILE> <yourfolder>` to download the output image

    public void testPipelineDrawingAndSave() throws Exception {
        // create pipeline obj
        TerrorBlobLocatorProcessor pipeline = TerrorCameraVisionPortal.initializeColorLocatorProcessor(
                new MatchColors(SampleColor.RED, SampleColor.BLUE)
        );
        CameraCalibration calibration = null; // TODO: add the actual calibration, for now we don't use it so it's fine

        // load in image from file, initialize pipeline
        InputStream input = InstrumentationRegistry.getInstrumentation().getContext().getAssets().open(INPUT_FILE);
        Bitmap bitmap = BitmapFactory.decodeStream(input);
        Mat frame = new Mat();
        Utils.bitmapToMat(bitmap, frame);
//        Mat frame = Imgcodecs.imread(INPUT_FILE);


        pipeline.init(frame.width(), frame.height(), calibration, true);

        Context context = InstrumentationRegistry.getInstrumentation().getTargetContext();
        pipeline.setContext(context);

        // run processframe()
        long captureTime = System.currentTimeMillis(); // fake capture time for testing
        Object blobs = pipeline.processFrame(frame, captureTime);


        // make the canvas
        Bitmap imgBitmap = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(frame, imgBitmap);

        int canvasWidth = imgBitmap.getWidth()*2;
        int canvasHeight = imgBitmap.getHeight()*2;
        Bitmap outputBitmap = Bitmap.createBitmap(canvasWidth, canvasHeight, Bitmap.Config.ARGB_8888);

        Canvas canvas = new Canvas(outputBitmap);
        canvas.drawBitmap(imgBitmap, null, new Rect(0, 0, canvasWidth, canvasHeight), null);


        // here these should both be 1.0 but if we have diff width/height for canvas it'll be different
        float scaleBmpPxToCanvasPx = (float) canvasWidth / (float) imgBitmap.getWidth();
//        float scaleBmpPxToCanvasPx = 1.0f;
        float scaleCanvasDensity = Resources.getSystem().getDisplayMetrics().density;
//        float scaleCanvasDensity = 1.0f;

        pipeline.onDrawFrame(canvas, canvasWidth, canvasHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, blobs);

//        final Bitmap[] result = new Bitmap[1];
//        final CountDownLatch latch = new CountDownLatch(1);
//
//        pipeline.getFrameBitmap(Continuation.createTrivial(bmp -> {
//            result[0] = bmp;
//            latch.countDown();
//        }));
//
//        try {
//            latch.await(); // Wait for the bitmap to be set
//        } catch (InterruptedException e) {
//            Thread.currentThread().interrupt(); // restore interrupt status
//        }
//        outputBitmap = result[0];

        // save the output image to a file
        saveBitmapToPictures(context, outputBitmap, "image_annotated_" + System.currentTimeMillis() + ".png");
    }

    private static void saveBitmapToPictures(Context context, Bitmap bitmap, String filename) {
        ContentValues values = new ContentValues();
        values.put(MediaStore.Images.Media.DISPLAY_NAME, filename);
        values.put(MediaStore.Images.Media.MIME_TYPE, "image/png");
        values.put(MediaStore.Images.Media.RELATIVE_PATH, Environment.DIRECTORY_PICTURES);

        Uri uri = context.getContentResolver().insert(MediaStore.Images.Media.EXTERNAL_CONTENT_URI, values);
        try (OutputStream out = context.getContentResolver().openOutputStream(uri)) {
            bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.flush();
            Log.i("SaveBitmap", "Saved image to Pictures using MediaStore");
            Log.i("SaveBitmap", "Image URI: " + uri.toString());
            Log.i("SaveBitmap", "Image Filename: " + filename);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}