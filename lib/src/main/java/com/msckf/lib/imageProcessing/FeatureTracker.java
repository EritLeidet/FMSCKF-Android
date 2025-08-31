package com.msckf.lib.imageProcessing;

import com.msckf.lib.msckfs.Fmsckf;
import com.msckf.lib.msckfs.Msckf;
import com.msckf.lib.msckfs.Odometry;
import com.msckf.lib.utils.Isometry3D;

import org.ddogleg.struct.DogArray_I8;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import boofcv.abst.feature.detect.interest.ConfigPointDetector;
import boofcv.abst.feature.detect.interest.PointDetectorTypes;
import boofcv.abst.tracker.PointTrack;
import boofcv.abst.tracker.PointTracker;
import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.alg.tracker.klt.ConfigPKlt;
import boofcv.factory.tracker.FactoryPointTracker;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.image.GrayS16;
import boofcv.struct.image.GrayU8;
import georegression.struct.point.Point2D_F64;

public class FeatureTracker {

    // TODO: RANSAC?

    // TODO: do you have to use imu as well, to help feature tracking?
    private final List<PointTrack> active = new LinkedList<>(); // TODO: if used concurrently, change maybe to CopyOnWriteArrayList!
    private final List<PointTrack> inactive = new LinkedList<>();
    private final PointTracker<GrayU8> tracker;


    private static final int respawnThreshold = 8; // based on f_Min Parameter in FMSCKF
    private static final int maxFeatures = 50; // 350


    private final Msckf msckf = new Fmsckf();

    public FeatureTracker(LensDistortionNarrowFOV lensDistortionModel) {
        // TODO: use configHarris
        this.undistorter = lensDistortionModel.undistort_F64(true, true);
        this.tracker = createTracker();
    }

    public PointTracker<GrayU8> createTracker() {
        ConfigPKlt configKlt = new ConfigPKlt();
        ConfigPointDetector configDetector = new ConfigPointDetector();
        configDetector.type = PointDetectorTypes.HARRIS;
        configDetector.general.maxFeatures = maxFeatures; // TODO: does this need to be the same as the other detector?

        return FactoryPointTracker.klt(configKlt, configDetector, GrayU8.class, GrayS16.class);


    }


    // TODO: warum ist langsamer als BoofCV Demo-App? Vielleicht mal im Produktionsmodus ausführen?

    private Point2Transform2_F64 undistorter;
    private List<FeatureMeasurement> undistortPoints(List<PointTrack> tracks) {
        List<FeatureMeasurement> undistortedPoints = new ArrayList<>(tracks.size());
        Point2D_F64 buffer = new Point2D_F64();
        for (PointTrack track : tracks) {
            undistorter.compute(track.pixel.x, track.pixel.y, buffer);
            undistortedPoints.add(new FeatureMeasurement(track.featureId, buffer.x, buffer.y));
        }
        return undistortedPoints;
        // TODO: normalize points? Tutorial mentions normalization, but only undistorts? What about MSCKF-S?
        // TODO: which points? New or all? ALL.
        // TODO: ...
    }


    public FeatureMessage trackFeatures(ImageMessage imageMessage) {
        if (tracker == null) return null;
        tracker.process(imageMessage.image);

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

        return new FeatureMessage(imageMessage.time, undistortPoints(active));



    }

}
