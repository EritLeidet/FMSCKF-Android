package com.android.msckfs.msckfs;

import org.apache.commons.collections4.map.LinkedMap;
import org.ejml.simple.SimpleMatrix;

import java.util.HashMap;
import java.util.Map;

public class StateServer {
    public final ImuState imuState = new ImuState();
    public final LinkedMap<Integer, CamState> camStates = new LinkedMap<>();
    public final Map<Long, Feature> mapServer = new HashMap<>();
    public SimpleMatrix stateCov; // State covariance matrix.
    public final SimpleMatrix continuousNoiseCov;

    public StateServer(SimpleMatrix stateCov, SimpleMatrix continuousNoiseCov) {
        this.stateCov = stateCov;
        this.continuousNoiseCov = continuousNoiseCov;
    }
}
