package com.msckf.lib.msckfs;

import static java.lang.Math.pow;

import com.msckf.lib.utils.Isometry3D;

import org.ejml.simple.SimpleMatrix;

/**
 * MSCKF configuration parameters that are based on user device.
 */
public class MsckfExternalConfig {
    final Isometry3D tCamImu; // camera to body transform

    // Noise related parameters (Use variance instead of standard deviation)
    final double gyroNoise;
    final double accNoise;

    final double gyroBiasNoise;
    final double accBiasNoise;

    public MsckfExternalConfig() {
        // Default parameters from official MSCKF implementation.
        this.tCamImu = new Isometry3D(SimpleMatrix.identity(3), new SimpleMatrix(3,1));
        this.gyroNoise = pow(0.005, 2);
        this.gyroBiasNoise = pow(0.001,2);
        this.accNoise = pow(0.05,2);
        this.accBiasNoise = pow(0.01,2);

    }
    public MsckfExternalConfig(Isometry3D tCamImu, double gyroNoise, double gyroBiasNoise, double accNoise, double accBiasNoise) {
        this.tCamImu = tCamImu;
        this.gyroNoise = gyroNoise;
        this.accNoise = accNoise;
        this.gyroBiasNoise = gyroBiasNoise;
        this.accBiasNoise = accBiasNoise;
    }

}
