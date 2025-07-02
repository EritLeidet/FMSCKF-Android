package com.android.msckfs;

import org.ejml.simple.SimpleMatrix;

import java.util.LinkedHashMap;
import java.util.Map;

public class StateServer {

    // TODO: initialize
    public ImuState imuState;
    public Map<Integer, CamState> camStates; // TODO: doch nicht? -> Iteration order should be insertion order. (Ascending by camState ID.)

    public Map<Integer, Feature> mapServer;
    public SimpleMatrix covariance;
}
