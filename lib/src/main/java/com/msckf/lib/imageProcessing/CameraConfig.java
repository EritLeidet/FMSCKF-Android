package com.msckf.lib.imageProcessing;

import com.msckf.lib.benchmark.BenchmarkCameraConfig;
import com.msckf.lib.benchmark.CameraInstrinsicParameters;
import com.msckf.lib.utils.Isometry3D;

import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.alg.distort.brown.LensDistortionBrown;
import boofcv.alg.distort.pinhole.LensDistortionPinhole;
import boofcv.struct.calib.CameraPinhole;
import boofcv.struct.calib.CameraPinholeBrown;

public class CameraConfig {

    public LensDistortionNarrowFOV lensDistortion;

    Isometry3D tCamImu; // camera to body transform // TODO




}
