package com.android.msckfs;


import com.msckf.lib.msckfs.Msckf;

import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

/**
 * Source code based on:
 * <a href="https://github.com/Edwinem/msckf_tutorial/blob/main/tests/test_msckf.py">...</a>
 */
public class MsckfUnitTest {


    @Test
    public void testMsckfImuIntegration() {
        Msckf msckf = new Msckf();

        double linearAcceleration = 0.3;
        double gravity = 9.81;

        SimpleMatrix accMeasurement = new SimpleMatrix(new double[]{
                linearAcceleration,
                0,
                gravity
        });
        SimpleMatrix gyroMeas = new SimpleMatrix(3,0);
        double dt = 1.0;

        // TODO: msckf.processModel(dt, gyroMeas, accMeasurement);

    }
}
