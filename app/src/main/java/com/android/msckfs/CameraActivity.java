package com.android.msckfs;

import static androidx.camera.core.resolutionselector.ResolutionStrategy.FALLBACK_RULE_CLOSEST_LOWER;

import android.Manifest;
import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.util.Size;
import android.view.View;

import androidx.appcompat.app.AppCompatActivity;
import androidx.camera.core.CameraSelector;
import androidx.camera.core.Preview;
import androidx.camera.core.UseCase;
import androidx.camera.core.UseCaseGroup;
import androidx.camera.core.resolutionselector.ResolutionSelector;
import androidx.camera.core.resolutionselector.ResolutionStrategy;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.camera.view.PreviewView;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.lifecycle.LifecycleOwner;

import com.android.msckfs.databinding.ActivityFeatureTrackerBinding;
import com.android.msckfs.databinding.ActivityMainBinding;
import com.google.common.util.concurrent.ListenableFuture;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Source code based on:
 *  - <a href="https://developer.android.com/codelabs/camerax-getting-started">...</a>
 */
public class CameraActivity extends AppCompatActivity {

    private static final String TAG = "CameraActivity";
    protected ActivityFeatureTrackerBinding viewBinding; // TODO: XML-Name not generic

    // Resolution.
    protected final ResolutionSelector resolutionSelector = new ResolutionSelector.Builder()
            .setResolutionStrategy(new ResolutionStrategy(new Size(640,480), FALLBACK_RULE_CLOSEST_LOWER))
            .build();


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        viewBinding = ActivityFeatureTrackerBinding.inflate(getLayoutInflater());
        setContentView(viewBinding.getRoot());
        requestCameraPermission();
        startCamera(); // TODO: only start Camera if permission granted

        // Tell the library what resolution you want. It searches for the resolution
        // which has the total number of pixels closest to this value. Most computer vision
        // algorithms run much faster and even work better at low resolutions.
        // TODO: keep aspect ratio or change?

    }


    private void startCamera() {
        // Used to bind the lifecycle of cameras to the lifecycle owner
        ListenableFuture<ProcessCameraProvider> cameraProviderFuture  = ProcessCameraProvider.getInstance(this);
        cameraProviderFuture.addListener(() -> {
            try {
                // Used to bind the lifecycle of cameras to the lifecycle owner
                ProcessCameraProvider cameraProvider = cameraProviderFuture.get();

                // Select back camera as a default
                CameraSelector cameraSelector = CameraSelector.DEFAULT_BACK_CAMERA;

                // Unbind use cases before rebinding
                cameraProvider.unbindAll();
                cameraProvider.bindToLifecycle(this, cameraSelector, getUseCases());


            } catch (ExecutionException | InterruptedException e) {
                Log.e(TAG, "Use case binding failed", e);
            }
        }, ContextCompat.getMainExecutor(this));

    }

    protected UseCaseGroup getUseCases() {
        UseCaseGroup.Builder useCases = new UseCaseGroup.Builder();
        useCases.addUseCase(getPreviewUseCase());
        return useCases.build();
    }

    protected UseCase getPreviewUseCase() {
        Preview preview = new Preview.Builder().setResolutionSelector(resolutionSelector).build();
        preview.setSurfaceProvider(viewBinding.viewFinder.getSurfaceProvider());
        return preview;
    }

    private void requestCameraPermission() {
        int permissionCheck = ContextCompat.checkSelfPermission(this, android.Manifest.permission.CAMERA);
        if (permissionCheck != android.content.pm.PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA}, 0);
            // TODO...?
        }

    }
}
