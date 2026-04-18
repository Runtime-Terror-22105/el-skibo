package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import android.annotation.SuppressLint;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.Log;

import androidx.annotation.ColorInt;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class BallDetectionPipeline extends ColorBlobLocatorProcessor implements VisionProcessor, CameraStreamSource
{
    //LOGIC
    //masks for color ranges, combine mask
    //do mask editing, dialation, erosion, etc
    //choose the contour one with the largest area
    //use a set multiplier to calculate pixels off center into inches off center
    //return position with the offset of the balls

    public static double PIXEL_TO_INCHES_SCALE = (double) 0.3; // pixels * 1/3 = inches

    // Configuration options
    public static boolean DRAW_CONTOURS = false;
    public static boolean DRAW_COORDS_TEXT = true;

    public static int COLOR_BIGGEST_BLOB = Color.GREEN;
    public static int COLOR_OTHER_BLOBS = Color.RED;

    private Rect roi; // roi = region of interest

    private int contourCode;

    private final Paint boundingRectPaint;
    private final Paint otherBlobsPaint;
    private final Paint biggestBlobPaint;
    private final Paint roiPaint;
    private final Paint contourPaint;
    private final @ColorInt int boundingBoxColor;
    private final @ColorInt int roiColor;
    private final @ColorInt int contourColor;

    private final Mat erodeElement;
    private final Mat dilateElement;
    private final Size blurElement;

    private final Object lockFilters = new Object();
    private final List<BlobFilter> filters = new ArrayList<>();
    private volatile BlobSort sort;

    private Mat temp = new Mat();
    private Mat roiMask = new Mat();
    private final Mat colorMask = new Mat();
    MatOfPoint maskShape;

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private final AtomicReference<Bitmap> lastMask =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    // for now I just set it to the full thing, but still have it as an option just in case
    public static Point[] ROI_POINTS = {
            new Point(40,100),
            new Point(40,140),
            new Point(50,140),
            new Point(50,180),
            new Point(280,180),
            new Point(250,70),
    };

    private boolean flipped = false;

    private final Object chosenBlobThreadLock = new Object();
    private BlobImpl chosenBlobSaved;
    private volatile BlobImpl chosenBlob;
    private boolean chosenBlobIsLocked;

    private Mat roiMat;

    private String TAG = "BallDetectionPipeline";
    private List<Blob> userBlobs;
    private int frameWidth;
    private int frameHeight;

    public enum StreamType {
        RAW, MASK, IMAGE_DRAWING
    }

    public static StreamType streamType = StreamType.MASK;

    public double pixelXtoRealX(double pixelX) {
        return (pixelX - ((double) frameWidth)/2) * PIXEL_TO_INCHES_SCALE;
    }

    public Point pixelToRealCoords(Point pixelCoords) {
        // TODO
        Point realCoords = new Point(pixelCoords.x, pixelCoords.y);
        realCoords.x = (realCoords.x - ((double) frameWidth)/2) * PIXEL_TO_INCHES_SCALE;
        return realCoords;
    }

    public Point realToPixelCoords(Point realCoords) {
        Point pixelCoords = new Point(realCoords.x, realCoords.y);
        pixelCoords.x = pixelCoords.x / PIXEL_TO_INCHES_SCALE + ((double) frameWidth)/2;
        return pixelCoords;
    }


    public BlobImpl getChosenBlob() {
        return chosenBlob;
        // todo: add back thread safety and locking the blob
//        synchronized (chosenBlobThreadLock) {
//            if (this.chosenBlob != null) {
//                this.chosenBlobIsLocked = true;
//                this.chosenBlobSaved = chosenBlob;
//            }
//            return chosenBlobSaved;
//        }
    }

    public void unlockChosenBlob() {
        synchronized (chosenBlobThreadLock) {
            this.chosenBlob = null;
            this.chosenBlobSaved = null;
            this.chosenBlobIsLocked = false;
        }
    }

    public BallDetectionPipeline(ContourMode contourMode,
                                      int erodeSize, int dilateSize, int blurSize,
                                      @ColorInt int boundingBoxColor, @ColorInt int roiColor, @ColorInt int contourColor)
    {
        Log.i(TAG, "Initializing BallDetectionPipeline");
        this.boundingBoxColor = boundingBoxColor;
        this.roiColor = roiColor;
        this.contourColor = contourColor;

        if (blurSize > 0) {
            // enforce Odd blurSize
            blurElement = new Size(blurSize | 0x01, blurSize | 0x01);
        } else {
            blurElement = null;
        }

        if (contourMode == ContourMode.EXTERNAL_ONLY) {
            contourCode = Imgproc.RETR_EXTERNAL;
        } else {
            contourCode = Imgproc.RETR_LIST;
        }

        if (erodeSize > 0) {
            erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erodeSize, erodeSize));
        } else {
            erodeElement = null;
        }

        if (dilateSize > 0) {
            dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilateSize, dilateSize));
        } else {
            dilateElement = null;
        }

        boundingRectPaint = new Paint();
        boundingRectPaint.setAntiAlias(true);
        boundingRectPaint.setStrokeCap(Paint.Cap.BUTT);
        boundingRectPaint.setColor(boundingBoxColor);

        otherBlobsPaint = new Paint(boundingRectPaint);
        otherBlobsPaint.setColor(COLOR_OTHER_BLOBS);
        biggestBlobPaint = new Paint(boundingRectPaint);
        biggestBlobPaint.setColor(COLOR_BIGGEST_BLOB);

        roiPaint = new Paint();
        roiPaint.setAntiAlias(true);
        roiPaint.setStyle(Paint.Style.STROKE);
        roiPaint.setStrokeCap(Paint.Cap.BUTT);
        roiPaint.setColor(roiColor);

        contourPaint = new Paint();
        contourPaint.setStyle(Paint.Style.STROKE);
        contourPaint.setColor(contourColor);

        this.chosenBlobIsLocked = false;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        lastMask.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));

        roiMask = new Mat(height, width, 0);

        maskShape = new MatOfPoint();
        maskShape.fromArray(ROI_POINTS);

        List<MatOfPoint> polygons = new ArrayList<>();
        polygons.add(maskShape);
        Imgproc.fillPoly(roiMask, polygons, new Scalar(255));

        this.frameWidth = width;
        this.frameHeight = height;

        synchronized (chosenBlobThreadLock) {
            chosenBlob = null;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Log.i(TAG, "Processing frame...");

//        if (frame == null || frame.empty()) {
//            Log.e(TAG, "Received empty frame, skipping processing.");
//            userBlobs = new ArrayList<>();
//            return userBlobs;
//        }
//
//        if (chosenBlobIsLocked) {
//            Log.i(TAG, "Locked on best sample at " + pixelToRealCoords(chosenBlob.getBoxFit().center));
//            userBlobs = new ArrayList<>();
//            return userBlobs;
//        }

//        // Flip and convert frame
//        Core.flip(frame, frame, -1);

        roiMat = frame.clone();
        Imgproc.cvtColor(roiMat, roiMat, Imgproc.COLOR_RGB2HSV);

        // Apply blur if configured
//        applyBlurIfNeeded();

        // Create masks for all color types
//        Core.inRange(this.roiMat, ColorRange.GREEN.min,  ColorRange.GREEN.max, colorMask);
        Core.inRange(this.roiMat, ColorRange.greenLow,  ColorRange.greenHigh, temp);
        Core.inRange(this.roiMat, ColorRange.purpleLow, ColorRange.purpleHigh, colorMask);
        Core.bitwise_or(colorMask, temp, colorMask);
//        Mat colorMask = createColorMask(ColorRange.GREEN, ColorRange.PURPLE_1, ColorRange.PURPLE_2);

        // Morphology cleans up the mask, erosion removes noise and dilation fills in gaps
        // TODO: I made the kernel size very big which is a little sus, might have to tune a bit, the reason is that nearby balls should be connected into a clump
        // even when there's a bit of a gap
        Size smallKernel = new Size(5, 5);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, smallKernel);
//        Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_CLOSE, kernel);

        // Apply ROI mask
        Core.bitwise_and(colorMask, roiMask, colorMask);
        Log.d(TAG, "Applied morphology operations and ROI masking");

        Bitmap maskBitmap = Bitmap.createBitmap(colorMask.width(), colorMask.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(colorMask, maskBitmap);
        lastMask.set(maskBitmap);

        // Find contours
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat(); // btw this can be ignored since we don't use tree
        Imgproc.findContours(colorMask, contours, hierarchy, contourCode, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();

        Log.d(TAG, "Found " + contours.size() + " contours");

        List<Blob> blobs = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            blobs.add(new BlobImpl(contour));
        }

        // Apply filters to all blobs
        applyFilters(blobs);

        if (blobs.isEmpty()) {
            colorMask.release();
            Log.w(TAG, "No valid blobs found after processing.");
            userBlobs = new ArrayList<>();
            return userBlobs;
        }

        Log.d(TAG, "Found " + blobs.size() + " BLOBS ");

        sort(blobs);
        chosenBlob = (BlobImpl) blobs.get(0);

        Bitmap outputBitmap = createOutputBitmap(frame, colorMask, blobs);
        lastFrame.set(outputBitmap);
        Log.d(TAG, "Made bitmap stuff");

        colorMask.release();

        userBlobs = new ArrayList<>(blobs);
        return userBlobs;
    }

    /**
     * Applies Gaussian blur to roiMat if blur element is configured
     */
    private void applyBlurIfNeeded() {
        if (blurElement != null) {
            Imgproc.GaussianBlur(roiMat, roiMat, blurElement, 0);
            Log.d(TAG, "Applied Gaussian blur to roiMat");
        }
    }

    /**
     * Creates a mask for the given color ranges
     */
    private Mat createColorMask(ColorRange... colorRanges) {
        Mat mask = new Mat();
        boolean first = true;

        for (ColorRange color : colorRanges) {
            if (first) {
                Core.inRange(this.roiMat, color.min, color.max, mask);
                first = false;
            } else {
                Core.inRange(this.roiMat, color.min, color.max, temp);
                Core.bitwise_or(mask, temp, mask);
            }
        }

        return mask;
    }

    /**
     * Applies all configured filters to the blob lists
     */
    private void applyFilters(List<Blob> blobs) {
        synchronized (lockFilters) {
            for (BlobFilter filter : filters) {
                Util.filterByCriteria(filter.criteria, filter.minValue, filter.maxValue, blobs);
            }
        }
    }

    private void sort(List<Blob> blobs) {
        if (sort != null) {
            Util.sortByCriteria(sort.criteria, sort.sortOrder, blobs);
        }
    }

    /**
     * Creates the output bitmap based on the current stream type
     */
    private Bitmap createOutputBitmap(@NonNull Mat frame, Mat colorMask, List<Blob> blobs) {
        Bitmap bmp = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);

        Log.d(TAG, "Creating output bitmap for stream type: " + streamType);
        switch (streamType) {
            case RAW:
                Utils.matToBitmap(frame, bmp);
                return bmp;
            case MASK:
                Utils.matToBitmap(colorMask, bmp);
                return bmp;
            case IMAGE_DRAWING:
                Utils.matToBitmap(frame, bmp);
                Canvas c = new Canvas(bmp);
                int w = bmp.getWidth();
                int h = bmp.getHeight();
                float scalePx = 1f;
                float density = Resources.getSystem().getDisplayMetrics().density;
                drawOverlay(c, w, h, scalePx, density, new ArrayList<>(blobs));
                return bmp;
            default:
                Log.e(TAG, "Unknown stream type: " + streamType);
                Utils.matToBitmap(frame, bmp);
                return bmp;
        }
    }

    private void drawOverlay(
            @NonNull Canvas canvas,
            int onscreenW,
            int onscreenH,
            float scaleBmpPxToCanvasPx,
            float scaleCanvasDensity,
            @NonNull ArrayList<Blob> blobs) {
        contourPaint.setStrokeWidth(scaleCanvasDensity * 0.6f);
        boundingRectPaint.setStrokeWidth(scaleCanvasDensity * 2f);
        roiPaint.setStrokeWidth(scaleCanvasDensity * 0.5f);

        Paint textPaint = new Paint();
        textPaint.setColor(contourColor);
        textPaint.setTextSize(scaleCanvasDensity * 5.0f);

        for (Blob blob : blobs) {
            if (DRAW_CONTOURS) {
                Path path = new Path();
                Point[] contourPts = blob.getContourPoints();
                path.moveTo((float) (contourPts[0].x) * scaleBmpPxToCanvasPx, (float) (contourPts[0].y) * scaleBmpPxToCanvasPx);
                for (int i = 1; i < contourPts.length; i++) {
                    path.lineTo((float) (contourPts[i].x) * scaleBmpPxToCanvasPx, (float) (contourPts[i].y) * scaleBmpPxToCanvasPx);
                }
                path.close();
                canvas.drawPath(path, contourPaint);
            }

            Point[] rotRectPts = new Point[4];
            blob.getBoxFit().points(rotRectPts);
            Paint thePaint;
            if (blob == chosenBlobSaved || (chosenBlobSaved == null && blob == chosenBlob)) thePaint = boundingRectPaint;
            else thePaint = biggestBlobPaint;

            for (int i = 0; i < 4; ++i) {
                canvas.drawLine(
                        (float) (rotRectPts[i].x) * scaleBmpPxToCanvasPx,
                        (float) (rotRectPts[i].y) * scaleBmpPxToCanvasPx,
                        (float) (rotRectPts[(i + 1) % 4].x) * scaleBmpPxToCanvasPx,
                        (float) (rotRectPts[(i + 1) % 4].y) * scaleBmpPxToCanvasPx,
                        thePaint
                );
            }

            if (DRAW_COORDS_TEXT && blob == chosenBlob) {
                Point centerPx = blob.getBoxFit().center;
//                Point centerIrl = pixelToRealCoords(centerPx);
                float pxX = (float) (centerPx.x * scaleBmpPxToCanvasPx);
                float pxY = (float) (centerPx.y * scaleBmpPxToCanvasPx);
                canvas.drawText(
                        String.format("px:(%.1f,%.1f)", centerPx.x, centerPx.y),
                        pxX, pxY,
                        textPaint
                );
//                canvas.drawText(
//                        String.format("irl:(%.1f,%.1f)", centerIrl.x, centerIrl.y),
//                        pxX, pxY + textPaint.getTextSize(),
//                        textPaint
//                );
            }
        }

        Path path = new Path();
        Point[] contourPts = ROI_POINTS;
        path.moveTo((float) (contourPts[0].x) * scaleBmpPxToCanvasPx, (float) (contourPts[0].y) * scaleBmpPxToCanvasPx);
        for (int i = 1; i < contourPts.length; i++) {
            path.lineTo((float) (contourPts[i].x) * scaleBmpPxToCanvasPx, (float) (contourPts[i].y) * scaleBmpPxToCanvasPx);
        }
        path.close();
        canvas.drawPath(path, roiPaint);
        //
//        // flip the camera
//        canvas.scale(-1f, -1f, canvas.getWidth() / 2f, canvas.getHeight() / 2f);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void onDrawFrame(@NonNull Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (canvas == null || userContext == null) {
            Log.e(TAG, "Canvas or userContext is null, skipping drawing.");
            return;
        }

        if (!(userContext instanceof ArrayList)) {
            Log.e(TAG, "userContext is not an ArrayList, skipping drawing.");
            return;
        }

        switch (streamType) {
            case MASK:
                canvas.drawBitmap(lastMask.get(), 0, 0, null);
                break;
            case IMAGE_DRAWING:
                drawOverlay(canvas,
                        onscreenWidth,
                        onscreenHeight,
                        scaleBmpPxToCanvasPx,
                        scaleCanvasDensity,
                        (ArrayList<Blob>) userContext);
                break;
            case RAW:
            default:
                // Do nothing, assume canvas has the raw frame
                break;
        }
    }

    @NonNull
    private android.graphics.Rect makeGraphicsRect(@NonNull Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void addFilter(BlobFilter filter) {
        synchronized (lockFilters) {
            filters.add(filter);
        }
    }

    @Override
    public void removeFilter(BlobFilter filter) {
        synchronized (lockFilters) {
            filters.remove(filter);
        }
    }

    @Override
    public void removeAllFilters() {
        synchronized (lockFilters) {
            filters.clear();
        }
    }

    @Override
    public void setSort(BlobSort sort) {
        this.sort = sort;
    }

    @Override
    public List<Blob> getBlobs() {
        return userBlobs;
    }

    @Override
    public void getFrameBitmap(@NonNull Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public void setFlipped(boolean flipped) {
        this.flipped = flipped;
    }

    static class BlobImpl extends Blob {
        private MatOfPoint contour;
        private Point[] contourPts;
        private int area = -1;
        private double density = -1;
        private double aspectRatio = -1;
        private RotatedRect rect;

        BlobImpl(MatOfPoint contour)
        {
            this.contour = contour;
        }

        @Override
        public MatOfPoint getContour()
        {
            return contour;
        }

        @Override
        public Point[] getContourPoints()
        {
            if (contourPts == null)
            {
                contourPts = contour.toArray();
            }

            return contourPts;
        }

        @Override
        public MatOfPoint2f getContourAsFloat()
        {
            return new MatOfPoint2f(getContourPoints());
        }

        @Override
        public int getContourArea()
        {
            if (area < 0)
            {
                area = Math.max(1, (int) Imgproc.contourArea(contour));  //  Fix zero area issue
            }

            return area;
        }

        public Pose2d getCenter(){
            Moments m = Imgproc.moments(contour);

            if (m.m00 != 0) {
                int cX = (int) (m.m10 / m.m00);
                int cY = (int) (m.m01 / m.m00);
                return new Pose2d(cX, cY);
            }
            return new Pose2d(0, 0);

        }

        @Override
        public double getDensity()
        {
            Point[] contourPts = getContourPoints();

            if (density < 0)
            {
                // Compute the convex hull of the contour
                MatOfInt hullMatOfInt = new MatOfInt();
                Imgproc.convexHull(contour, hullMatOfInt);

                // The convex hull calculation tells us the INDEX of the points which
                // which were passed in eariler which form the convex hull. That's all
                // well and good, but now we need filter out that original list to find
                // the actual POINTS which form the convex hull
                Point[] hullPoints = new Point[hullMatOfInt.rows()];
                List<Integer> hullContourIdxList = hullMatOfInt.toList();

                for (int i = 0; i < hullContourIdxList.size(); i++)
                {
                    hullPoints[i] = contourPts[hullContourIdxList.get(i)];
                }

                double hullArea = Math.max(1.0,Imgproc.contourArea(new MatOfPoint(hullPoints)));  //  Fix zero area issue

                density = getContourArea() / hullArea;
            }
            return density;
        }

        @Override
        public double getAspectRatio()
        {
            if (aspectRatio < 0)
            {
                RotatedRect r = getBoxFit();

                double longSize  = Math.max(1, Math.max(r.size.width, r.size.height));
                double shortSize = Math.max(1, Math.min(r.size.width, r.size.height));

                aspectRatio = longSize / shortSize;
            }

            return aspectRatio;
        }

        @Override
        public RotatedRect getBoxFit()
        {
            if (rect == null)
            {
                rect = Imgproc.minAreaRect(new MatOfPoint2f(getContourPoints()));

                if (rect.size.width < rect.size.height) {
                    double tmp = rect.size.width;             // swap width and height if wrong
                    rect.size.width  = rect.size.height;
                    rect.size.height = tmp;
                    rect.angle += 90;
                }
            }
            return rect;
        }

        @Override
        public double getArcLength()
        {
            return Imgproc.arcLength(getContourAsFloat(), true);
        }

        @Override
        public double getCircularity()
        {
            double area = getContourArea();
            double perimeter = getArcLength();
            if (perimeter == 0) return 0;
            return 4 * Math.PI * area / (perimeter * perimeter);
        }

        /**
         * Get the center Point and radius of the circle enclosing this blob
         * @return the center Point and radius of the circle enclosing this blob
         */
        @Override
        public Circle getCircle()
        {
            // note that both center and radius are output variables that the third line fills in for us
            Point center = new Point();
            float[] radius = new float[1];
            Imgproc.minEnclosingCircle(getContourAsFloat(), center, radius);
            return new Circle(center, radius[0]);
        }

        // Angle is normalized to be between -180 and 180 degrees
        public double getAngleLongEdgeNotSus() {
            return AngleUnit.normalizeDegrees(rect.angle + 90);
        }

        // see https://stackoverflow.com/a/21427814/13138596
        private double getAngleShortEdge() {
            RotatedRect rect = getBoxFit();
            if (rect.size.width < rect.size.height) {
                return rect.angle + 180;
            } else {
                return rect.angle + 90;
            }
        }

        private double getAngleLongEdge() {
            RotatedRect rect = getBoxFit();
            if (rect.size.width >= rect.size.height) {
                return rect.angle;
            } else {
                return AngleUnit.normalizeDegrees(rect.angle + 90);
            }
        }

        private double getPerpendicularAngleLongEdge() {
            return AngleUnit.normalizeDegrees(getAngleLongEdge() + 90);
        }

        public double getNecessaryTurretAngle() {
            double ang = this.getAngleLongEdgeNotSus();

            if (ang > 0) {
                return ang - 90;
            } else {
                return ang + 90;
            }
        }
    }
}
