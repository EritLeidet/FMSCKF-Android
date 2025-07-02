package com.android.msckfs;

/*
    Static parameters for the EKF.
 */
public class FilterParams {

    // Min and Max num of features used in EKF update
    public static final int minTrackLength = 3;
    public static final int maxTrackLength = 3;
}
