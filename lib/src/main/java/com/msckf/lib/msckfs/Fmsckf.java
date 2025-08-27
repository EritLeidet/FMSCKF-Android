package com.msckf.lib.msckfs;

import org.ejml.simple.SimpleMatrix;

import java.util.LinkedList;
import java.util.List;

// TODO: quote FMSCKF paper
public class Fmsckf extends Msckf {

    @Override
    public boolean fmsckfUpdate() {
        if (stateServer.mapServer.size() >= Config.MIN_TRACKED_FEATURES) return false;

        // TODO: do anything with these?
        List<Long> invalidFeatureIds = new LinkedList<>();
        List<Feature> processedFeatures = new LinkedList<>();

        int jacobianRowSize = 0;
        for (Feature feature : stateServer.mapServer.values()) {
            // TODO: ...

            if (!feature.isInitialized) {
                if (feature.observations.size() < 3) {
                    invalidFeatureIds.add(feature.id);
                    continue;
                }
                // Ensure there is enough translation to triangulate the feature
                if (!feature.checkMotion(stateServer.camStates)) {
                    // If the feature cannot be initialized, just remove
                    // the observations associated with the camera states
                    // to be removed.
                    invalidFeatureIds.add(feature.id);
                    continue;
                }

                // Intialize the feature position based on all current available measurements.
                boolean ret = feature.initializePosition(stateServer.camStates);
                if (!ret) {
                    invalidFeatureIds.add(feature.id);
                    continue;
                }
            }
            jacobianRowSize += 2 * feature.observations.size() - 3;
            processedFeatures.add(feature);
        }
        // Remove the features that do not have enough measurements.
        for (Long featureId : invalidFeatureIds) {
            stateServer.mapServer.remove(featureId);
        }

        // Return if there is no lost feature to be processed.
        if (processedFeatures.isEmpty()) return true;



        SimpleMatrix Hx = new SimpleMatrix(jacobianRowSize, getStateSize());
        SimpleMatrix r = new SimpleMatrix(jacobianRowSize,1);
        int stackCount = 0;
        SimpleMatrix Hxj = new SimpleMatrix(0,0);
        SimpleMatrix rj = new SimpleMatrix(0,0);


        for (Feature feature : processedFeatures) {
            assert(!feature.observations.isEmpty());
            featureJacobian(feature, feature.observations.asList(), Hxj, rj);

            if (gatingTest(Hxj, rj, feature.observations.size()-1)) {
                Hx.insertIntoThis(stackCount, 0, Hxj);
                r.insertIntoThis(stackCount, 0, rj);
                stackCount += Hxj.getNumRows();
            }
            // Put an upper bound on the row size of measurement Jacobian,
            // which helps guarantee the execution time.
            if (stackCount > 1500) break;
        }
        Hx.reshape(stackCount, Hx.getNumCols());
        r.reshape(stackCount, 1);

        // Perform the measurement update step.
        measurementUpdate(Hx, r);

        // Prune all images except the latest.
        if (stateServer.camStates.size() > 1) {
            pruneCamStates(stateServer.camStates.asList().subList(0,stateServer.camStates.size()-1));

        }

        // TODO: delete processed features or nah?
        return true;

    }
    private static class Config {

        static final int MIN_TRACKED_FEATURES = 8; // 8 is min. if you use RANSAC. See: FMSCKF Paper.

    }
}
