package com.android.msckfs;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.util.Log;

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
import java.util.Iterator;
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
    private static final int maxFeatures = 350;

    // TODO: didn't I need to implement some kind of distortion?

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        initTracker();



    }

    @Override
    protected UseCaseGroup getUseCases() {
        UseCaseGroup.Builder useCases = new UseCaseGroup.Builder();
        useCases.addUseCase(getPreviewUseCase());
        useCases.addUseCase(getAnalyzerUseCase());
        useCases.addEffect(effect);
        return useCases.build();
    }


    private UseCase getAnalyzerUseCase() {
        // create overlay effect for feature visualization
        handlerThread = new HandlerThread("OverlayEffect thread"); // Gemini suggested to use not use effect on the main thread. Reduced GL Error 0x505 (GL_OUT_OF_MEMORY) warning.
        handlerThread.start();

        Handler handler = new Handler(handlerThread.getLooper());
        Consumer<Throwable> errorListener = throwable -> {
            Log.e(TAG, "Visualization ran into unrecoverable error", throwable);
        };


        effect = new OverlayEffect(CameraEffect.PREVIEW,0, handler, errorListener); // TODO: queueDepth?

        Visualizer visualizer = new Visualizer();
        // TODO: add to CameraProviderFuture
        effect.setOnDrawListener(frame -> {
            visualizer.onDraw( frame.getOverlayCanvas());
            return true;
        });


        cameraExecutor = Executors.newSingleThreadExecutor();
        ImageAnalysis imageAnalysis = new ImageAnalysis.Builder().build(); // TODO: further modify (e.g. resolution) for better frame rate.
        imageAnalysis.setAnalyzer(cameraExecutor, this);
        return imageAnalysis;
    }



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
        private final Paint paint = new Paint();

        public Visualizer() {
            paint.setColor(Color.RED);
            paint.setStyle(Paint.Style.FILL);
        }

        public void onDraw(Canvas canvas) {
            for (PointTrack track : active) {
                canvas.drawCircle((float) track.pixel.x, (float) track.pixel.y, 5, paint); // TODO: the coordinates are nonsense. Look at the BoofCV demo code.
            }
            // canvas.drawCircle(canvas.getWidth() / 2f, canvas.getHeight() / 2f, 100, paint);
            //canvas.drawPoint(rand.nextInt(canvas.getWidth()), rand.nextInt(canvas.getWidth()), paint);

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