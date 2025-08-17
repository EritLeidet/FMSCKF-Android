package com.android.msckfs.imageProcessing;

import static android.hardware.camera2.CameraCharacteristics.LENS_DISTORTION;
import static android.hardware.camera2.CameraCharacteristics.LENS_INTRINSIC_CALIBRATION;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.graphics.Rect;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;

import androidx.annotation.OptIn;
import androidx.camera.camera2.interop.ExperimentalCamera2Interop;
import androidx.camera.core.CameraEffect;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import androidx.camera.core.UseCase;
import androidx.camera.core.UseCaseGroup;
import androidx.camera.effects.OverlayEffect;
import androidx.core.util.Consumer;

import com.android.msckfs.imuProcessing.ImuProcessor;
import com.android.msckfs.msckfs.Fmsckf;
import com.android.msckfs.msckfs.Msckf;
import com.android.msckfs.msckfs.Odometry;
import com.android.msckfs.utils.Isometry3D;

import org.ddogleg.struct.DogArray_I8;
import org.ejml.data.FMatrixRMaj;
import org.ejml.simple.SimpleMatrix;
import org.jspecify.annotations.NonNull;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import boofcv.abst.feature.detect.interest.ConfigPointDetector;
import boofcv.abst.feature.detect.interest.PointDetectorTypes;
import boofcv.abst.tracker.PointTrack;
import boofcv.abst.tracker.PointTracker;
import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.alg.distort.brown.LensDistortionBrown;
import boofcv.alg.tracker.klt.ConfigPKlt;
import boofcv.android.ConvertBitmap;
import boofcv.factory.tracker.FactoryPointTracker;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.image.GrayS16;
import boofcv.struct.image.GrayU8;
import georegression.struct.point.Point2D_F64;

/**
 * Source code based on:
 *  - <a href="https://github.com/lessthanoptimal/BoofAndroidDemo">...</a>
 *  - <a href="https://developer.android.com/codelabs/camerax-getting-started">...</a>
 *  - <a href="https://github.com/lessthanoptimal/AndroidAutoFocus/blob/master/app/src/main/java/boofcv/androidautofocus/MainActivity.java">...</a>
 *  - https://android-review.googlesource.com/c/platform/frameworks/support/+/2797834/9/camera/integration-tests/viewtestapp/src/main/java/androidx/camera/integration/view/OverlayFragment.kt#90
 */
public class Vio extends CameraActivity implements ImageAnalysis.Analyzer {

    // TODO: RANSAC?

    // TODO: do you have to use imu as well, to help feature tracking?
    private static final String TAG = "VIO";
    private final List<PointTrack> active = new CopyOnWriteArrayList<>(); // TODO: if not used concurrently anymore, change back to a normal ArrayList for performance reasons.
    private final List<PointTrack> spawned = new ArrayList<>();
    private final List<PointTrack> inactive = new ArrayList<>();
    private PointTracker<GrayU8> tracker;

    private ExecutorService cameraExecutor;

    private HandlerThread handlerThread;
    private Handler handler;
    private OverlayEffect effect;


    private static final int respawnThreshold = 50; // if number of tracks drops below this value it will attempt to spawn more
    private static final int maxFeatures = 200; // 350


    private final Msckf msckf = new Fmsckf();

    private ImuProcessor imuProcessor;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.d(TAG, "Created FeatureTracker");
        initTracker();
        try {
            this.undistort = createUndistort();
            this.imuProcessor = new ImuProcessor(getApplicationContext(), msckf);
        } catch (CameraAccessException e) {
            Log.e(TAG, "Camera access denied.");
        }


    }

    // TODO: warum ist langsamer als BoofCV Demo-App? Vielleicht mal im Produktionsmodus ausführen?

    @Override
    protected UseCaseGroup getUseCases() {
        UseCaseGroup.Builder useCases = new UseCaseGroup.Builder();
        useCases.addUseCase(getAnalyzerUseCase());
        useCases.addUseCase(getPreviewUseCase());
        useCases.addEffect(effect);
        return useCases.build();
    }

    private Point2Transform2_F64 undistort;
    private Point2Transform2_F64 createUndistort() throws CameraAccessException {
        CameraManager cameraManager = (CameraManager) getApplicationContext().getSystemService(Context.CAMERA_SERVICE);
        CameraCharacteristics cameraCharacteristics = cameraManager.getCameraCharacteristics("0"); // TODO: determine cameraID
        float[] lensIntrinsics = cameraCharacteristics.get(LENS_INTRINSIC_CALIBRATION);
        float[] distortionParams = cameraCharacteristics.get(LENS_DISTORTION);
        CameraPinholeBrown cameraModel = new CameraPinholeBrown(
                lensIntrinsics[0], // fx
                lensIntrinsics[1], // fy
                lensIntrinsics[4], // skew
                lensIntrinsics[2], // cx
                lensIntrinsics[3], // cy
                resolution.getWidth(),
                resolution.getHeight());
        cameraModel.setRadial(
                distortionParams[0], // kappa_1
                distortionParams[1], // kappa_2
                distortionParams[2]); // kappa_3
        cameraModel.setT1(distortionParams[3]);
        cameraModel.setT2(distortionParams[4]);
        LensDistortionNarrowFOV lensUndistorter = new LensDistortionBrown(cameraModel);

        return lensUndistorter.undistort_F64(true, true);

    }
    private List<FeatureMeasurement> undistortPoints(List<PointTrack> tracks) {
        List<FeatureMeasurement> undistortedPoints = new ArrayList<>(tracks.size());
        Point2D_F64 buffer = new Point2D_F64();
        for (PointTrack track : tracks) {
            undistort.compute(track.pixel.x, track.pixel.y, buffer);
            undistortedPoints.add(new FeatureMeasurement(track.featureId, buffer.x, buffer.y));
        }
        return undistortedPoints;
        // TODO: normalize points? Tutorial mentions normalization, but only undistorts? What about MSCKF-S?
        // TODO: which points? New or all? ALL.
        // TODO: ...
    }


    @OptIn(markerClass = ExperimentalCamera2Interop.class)
    private UseCase getAnalyzerUseCase() {
        // create overlay effect for feature visualization
        handlerThread = new HandlerThread("OverlayEffect thread"); // Gemini suggested to use not use effect on the main thread. Reduced GL Error 0x505 (GL_OUT_OF_MEMORY) warning.
        handlerThread.start();

        Handler handler = new Handler(handlerThread.getLooper());
        Consumer<Throwable> errorListener = throwable -> {
            Log.e(TAG, "Visualization ran into unrecoverable error", throwable);
        };


        effect = new OverlayEffect(CameraEffect.PREVIEW,8, handler, errorListener); // TODO: queueDepth? // if queueDepth too low, overlayEffect lags behind camera preview

        Visualizer visualizer = new Visualizer();
        // TODO: add to CameraProviderFuture
        effect.setOnDrawListener(frame -> {
            visualizer.onDraw( frame.getOverlayCanvas());
            return true;
        });



        cameraExecutor = Executors.newSingleThreadExecutor();
        ImageAnalysis.Builder builder = new ImageAnalysis.Builder();

        // Set resolution.
        builder.setResolutionSelector(resolutionSelector);

        // Set target frame rate.
        // TODO: once bugs are solved, remove this part to go back to (higher) default frame rate.
        /*
        Camera2Interop.Extender<ImageAnalysis> ext = new Camera2Interop.Extender<>(builder);
        ext.setCaptureRequestOption(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF);
        ext.setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, new Range<>(20, 20));
         */

        // Build Use Case.
        ImageAnalysis imageAnalysis = builder.build();
        imageAnalysis.setAnalyzer(cameraExecutor, this);
        return imageAnalysis;
    }

    // TODO: private ResolutionStrategy resolutionStrategy = new ResolutionStrategy();


    private final DogArray_I8 analyzeBuffer = new DogArray_I8();

    private final Odometry odom = new Odometry(
            -1,
            new Isometry3D(SimpleMatrix.identity(3), new SimpleMatrix(3,1)),
            new SimpleMatrix(3,1),
            new Isometry3D(SimpleMatrix.identity(3), new SimpleMatrix(3,1)));

    @Override
    public void analyze(@NonNull ImageProxy imageProxy) {

        if (tracker == null) return;
        GrayU8 image = ConvertBitmap.bitmapToGray(imageProxy.toBitmap(),(GrayU8)null,analyzeBuffer);

        tracker.process(image);

        // drop tracks which are no longer being used
        tracker.getInactiveTracks(inactive);
        for (PointTrack track : inactive) {
            if(tracker.getFrameID() - track.lastSeenFrameID > 2) tracker.dropTrack(track);
        }

        tracker.getActiveTracks(active);

        //spawned.clear();
        if (active.size() < respawnThreshold ) {
            tracker.spawnTracks();
            //tracker.getNewTracks(spawned);

        }


        effect.drawFrameAsync(imageProxy.getImageInfo().getTimestamp()); // TODO: why does it work (but is slower) when removing this line?


        Odometry latestOdom = msckf.featureCallback(new FeatureMessage(
                imageProxy.getImageInfo().getTimestamp(),
                undistortPoints(active)));
        if (latestOdom != null) {
            synchronized (this.odom) {
                this.odom.setTo(latestOdom);
            }
            Log.i(TAG, latestOdom.pose.toString());
        }




        imageProxy.close();

    }


    /**
     * Source code: https://stackoverflow.com/questions/21221649/android-how-to-use-the-ondraw-method-in-a-class-extending-activity
     */
     private class Visualizer {
        private final Random rand = new Random();
        private List<Paint> paints;

        public Visualizer() {
            paints = new ArrayList<>();

            int[] colors = new int[] {
              Color.RED,
              Color.BLUE,
              Color.CYAN,
              Color.GREEN,
              Color.MAGENTA,
              Color.YELLOW,
              Color.WHITE
            };

            for (int color : colors) {
                Paint paint = new Paint();
                paint.setColor(color);
                paint.setStyle(Paint.Style.STROKE);
                paint.setStrokeWidth(8);
                paints.add(paint);
            }

            this.positionDiagramPaint = new Paint();
            this.positionDiagramPaint.setColor(Color.WHITE);
            this.positionDiagramPaint.setStyle(Paint.Style.STROKE);
            this.positionDiagramPaint.setStrokeWidth(2);


            this.axisPaints = new Paint[]{new Paint(),new Paint(),new Paint()};
            for (Paint axisPaint : axisPaints) {
                axisPaint.setStyle(Paint.Style.STROKE);
                axisPaint.setStrokeWidth(1);
            }
            axisPaints[0].setColor(Color.RED);
            axisPaints[1].setColor(Color.GREEN);
            axisPaints[2].setColor(Color.BLUE);
        }


        private final SimpleMatrix translation = new SimpleMatrix(new float[]{300,300,300});

        private static final float axisLength = 150;
        private final SimpleMatrix xAxis = new SimpleMatrix(new float[]{axisLength,0,0});
        private final SimpleMatrix yAxis = new SimpleMatrix(new float[]{0,-axisLength,0});
        private final SimpleMatrix zAxis = new SimpleMatrix(new float[]{0,0,axisLength});



        private final Paint[] axisPaints;

        private final Paint positionDiagramPaint;

        private void drawPosition(Canvas canvas) {

            // Draw outline.
            final float diagramX = 400, diagramY = 200, diagramSize = 100; // TODO: correct scale bc. currently playerPosition is in meters, but Size is in pixels.
            canvas.drawRect(diagramX, diagramY,diagramX+diagramSize,diagramY+diagramSize, positionDiagramPaint);

            // Draw player X-Z position.
            final float playerX, playerY, playerZ;
            synchronized (odom) {
                playerX = (float) odom.pose.t.get(0);
                playerY = (float) odom.pose.t.get(1);
                playerZ = (float) odom.pose.t.get(2);
            }
            canvas.drawCircle(diagramX + (diagramSize/2) + playerX, diagramY + (diagramSize/2) + playerZ, 5, paints.get(1));


            // Draw player height
            final float heightDiagramX = diagramX+diagramSize+diagramSize/2;
            canvas.drawLine(heightDiagramX, diagramY, heightDiagramX, diagramY + diagramSize, positionDiagramPaint);
            canvas.drawCircle(heightDiagramX, diagramY + (diagramSize/2) + playerY, 5, paints.get(1));





        }
        private void drawOrientation(Canvas canvas) {
            // Draw coordinate system on preview.


            FMatrixRMaj endXpoint, endYpoint, endZpoint;
            synchronized (odom) {
                endXpoint = (odom.pose.R).mult(xAxis).plus(translation).getFDRM();
                endYpoint = (odom.pose.R).mult(yAxis).plus(translation).getFDRM();
                endZpoint = (odom.pose.R).mult(zAxis).plus(translation).getFDRM();
            }

            // Draw axes.
            canvas.rotate(-90F, (float) translation.get(0), (float) translation.get(1));

            canvas.drawLine((float) translation.get(0), (float) translation.get(1), endXpoint.get(0), endXpoint.get(1), axisPaints[0]);
            canvas.drawLine((float) translation.get(0), (float) translation.get(1), endYpoint.get(0), endYpoint.get(1), axisPaints[1]);
            canvas.drawLine((float) translation.get(0), (float) translation.get(1), endZpoint.get(0),endZpoint.get(1), axisPaints[2]);



            // Label axes
            canvas.drawText("x", endXpoint.get(0), endXpoint.get(1), axisPaints[0]);
            canvas.drawText("y", endYpoint.get(0), endYpoint.get(1), axisPaints[1]);
            canvas.drawText("z", endZpoint.get(0), endZpoint.get(1), axisPaints[2]);

            canvas.rotate(90F, (float) translation.get(0), (float) translation.get(1));


        }

        private void drawFeatures(Canvas canvas) {
            for (PointTrack track : active) {
                canvas.drawCircle((float) track.pixel.x, (float) track.pixel.y, 5, paints.get((int) (track.featureId % paints.size()))); // TODO: the coordinates are nonsense. Look at the BoofCV demo code.
            }

        }
        public void onDraw(Canvas canvas) {
            // Clear canvas
            canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.MULTIPLY);

            //drawFeatures(canvas);
            drawOrientation(canvas);
            drawPosition(canvas);










        }

    }

    private void initTracker() {
        ConfigPKlt configKlt = new ConfigPKlt();
        ConfigPointDetector configDetector = new ConfigPointDetector();
        configDetector.type = PointDetectorTypes.HARRIS;
        // TODO: use configHarris
        configDetector.general.maxFeatures = maxFeatures; // TODO: does this need to be the same as the other detector?
        tracker = FactoryPointTracker.klt(configKlt, configDetector, GrayU8.class, GrayS16.class);

    }


    @Override
    protected void onDestroy() {
        super.onDestroy();
        effect.close();
        handlerThread.quitSafely();
        cameraExecutor.shutdown();
        imuProcessor.stop();
    }
}