package com.android.msckfs;

import org.apache.commons.collections4.map.LinkedMap;
import org.ejml.simple.SimpleMatrix;

import java.util.LinkedHashMap;
import java.util.Map;

public class StateServer {

    // TODO: initialize
    public ImuState imuState;
    public LinkedMap<Integer, CamState> camStates; // TODO: doch nicht? -> Iteration order should be insertion order. (Ascending by camState ID.)

    public Map<Integer, Feature> mapServer;
    public SimpleMatrix covariance;
}
