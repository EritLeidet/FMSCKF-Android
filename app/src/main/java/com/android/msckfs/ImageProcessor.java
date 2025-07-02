package com.android.msckfs;

import java.util.ArrayList;
import java.util.List;

import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.feature.detect.interest.ConfigHarrisCorner;
import boofcv.abst.feature.detect.interest.ConfigPointDetector;
import boofcv.abst.feature.detect.interest.PointDetectorTypes;
import boofcv.abst.tracker.PointTrack;
import boofcv.abst.tracker.PointTracker;
import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.alg.distort.brown.LensDistortionBrown;
import boofcv.alg.feature.detect.interest.GeneralFeatureDetector;
import boofcv.alg.sfm.d3.VisOdomDualTrackPnP;
import boofcv.alg.tracker.klt.ConfigPKlt;
import boofcv.factory.feature.detect.interest.FactoryDetectPoint;
import boofcv.factory.feature.detect.interest.FactoryInterestPoint;
import boofcv.factory.tracker.FactoryPointTracker;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.ImageBase;
import boofcv.struct.image.ImageGray;
import georegression.struct.point.Point2D_F64;

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
