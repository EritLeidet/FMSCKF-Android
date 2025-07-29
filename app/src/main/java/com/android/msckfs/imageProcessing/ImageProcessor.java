package com.android.msckfs.imageProcessing;

import boofcv.abst.feature.detect.interest.ConfigHarrisCorner;
import boofcv.struct.calib.CameraPinholeBrown;

// Pro tip: Run in app fast or release mode for a dramatic speed up!
// In Android Studio expand "Build Variants" tab on left.

// based on BoofCV Android demo: https://github.com/lessthanoptimal/BoofAndroidDemo/tree/SNAPSHOT
public class ImageProcessor {

    // TODO: feature grid / bucketing?

    private ConfigHarrisCorner configHarris = new ConfigHarrisCorner(); // default config through parameter-less constructor

    private final CameraPinholeBrown cameraModel;
    public ImageProcessor(CameraPinholeBrown cameraModel) {
        this.cameraModel = cameraModel;
    }








    /* // TODO

    public <T extends ImageGray<T>> FactoryInterestPoint createFeatureDetector(Class<T> imageType) {
        Class derivType = ;
        ConfigGeneralDetector configDetector = new ConfigGeneralDetector(); // TODO: Params?
        GeneralFeatureDetector featureDetector = FactoryDetectPoint.createHarris(configDetector, new ConfigHarrisCorner(), imageType); // TODO: derivType?
        featureDetector.setFeatureLimit(200);
        return FactoryInterestPoint.wrapPoint(featureDetector, )
    }

    public List<Point2D_F64> distortPoints(List<Point2D_F64> points) { // TODO: _32 also ok
        LensDistortionNarrowFOV undistorter = new LensDistortionBrown(cameraModel);
        Point2Transform2_F64 transform = undistorter.undistort_F64(TODO, false); // TODO: Are the input points already normalized?

        List<Point2D_F64> normalizedKeypoints = new ArrayList<>(points.size());
        Point2D_F64 norm = new Point2D_F64();
        for (Point2D_F64 pt : points) {
            transform.compute(pt.x, pt.y, norm);
            normalizedKeypoints.add(norm);
        }
        // TODO: apply concurrent?

        return normalizedKeypoints;
    }

     */
}
