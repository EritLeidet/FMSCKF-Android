package com.android.msckfs;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.hardware.camera2.CaptureRequest;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.util.Range;

import androidx.annotation.OptIn;
import androidx.camera.camera2.interop.Camera2Interop;
import androidx.camera.camera2.interop.ExperimentalCamera2Interop;
import androidx.camera.core.CameraEffect;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import androidx.camera.core.UseCase;
import androidx.camera.core.UseCaseGroup;
import androidx.camera.effects.OverlayEffect;
import androidx.core.util.Consumer;

import org.ddogleg.struct.DogArray_I8;
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
import boofcv.alg.tracker.klt.ConfigPKlt;
import boofcv.android.ConvertBitmap;
import boofcv.factory.tracker.FactoryPointTracker;
import boofcv.struct.image.GrayS16;
import boofcv.struct.image.GrayU8;

/**
 * Source code based on:
 *  - <a href="https://github.com/lessthanoptimal/BoofAndroidDemo">...</a>
 *  - <a href="https://developer.android.com/codelabs/camerax-getting-started">...</a>
 *  - <a href="https://github.com/lessthanoptimal/AndroidAutoFocus/blob/master/app/src/main/java/boofcv/androidautofocus/MainActivity.java">...</a>
 *  - https://android-review.googlesource.com/c/platform/frameworks/support/+/2797834/9/camera/integration-tests/viewtestapp/src/main/java/androidx/camera/integration/view/OverlayFragment.kt#90
 */
public class FeatureTracker extends CameraActivity implements ImageAnalysis.Analyzer {

    // TODO: RANSAC

    // TODO: do you have to use imu as well, to help feature tracking?
    private static final String TAG = "FeatureTracker";
    private final List<PointTrack> active = new CopyOnWriteArrayList<>(); // TODO: if not used concurrently anymore, change back to a normal ArrayList for performance reasons.
    private final List<PointTrack> spawned = new ArrayList<>();
    private final List<PointTrack> inactive = new ArrayList<>();
    private PointTracker<GrayU8> tracker;

    private ExecutorService cameraExecutor; // TODO: why attribute? Do I need to close?

    private HandlerThread handlerThread;
    private Handler handler;
    private OverlayEffect effect;


    private static final int respawnThreshold = 50; // if number of tracks drops below this value it will attempt to spawn more
    private static final int maxFeatures = 200; // 350

    // TODO: didn't I need to implement some kind of distortion?

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        initTracker();



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
        // Camera2Interop.Extender<ImageAnalysis> ext = new Camera2Interop.Extender<>(builder);
        // ext.setCaptureRequestOption(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF);
        // ext.setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, new Range<>(30, 30));

        // Build Use Case.
        ImageAnalysis imageAnalysis = builder.build();
        imageAnalysis.setAnalyzer(cameraExecutor, this);
        return imageAnalysis;
    }

    // TODO: private ResolutionStrategy resolutionStrategy = new ResolutionStrategy();


    private final DogArray_I8 analyzeBuffer = new DogArray_I8();

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

        // TODO: Nullpointer exception bc. FrameBuffer is null
        effect.drawFrameAsync(imageProxy.getImageInfo().getTimestamp());
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
              Color.YELLOW
            };

            for (int color : colors) {
                Paint paint = new Paint();
                paint.setColor(color);
                paint.setStyle(Paint.Style.FILL);
                paints.add(paint);
            }

        }


        public synchronized void onDraw(Canvas canvas) { // TODO: remove synchronized tag?
            // Debugging: duplicate features? Nope.

            // Clear canvas
            canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.MULTIPLY);
            /*
            if (firstDraw) {
                canvas.drawCircle(200, 200, 20, paints.get(0));
                firstDraw = false;
            }

             */

            canvas.drawCircle(canvas.getWidth(), canvas.getHeight(), 5, paints.get(0));


            for (PointTrack track : active) {
                canvas.drawCircle((float) track.pixel.x, (float) track.pixel.y, 5, paints.get((int) (track.featureId % paints.size()))); // TODO: the coordinates are nonsense. Look at the BoofCV demo code.
            }


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
    }
}