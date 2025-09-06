package com.msckf.lib.benchmark.datatypes.imu;

public class DiscreteNoiseParameters {

    // calibrated discrete noise parameters
    float sigma_w;
    float sigma_bw;
    float sigma_a;
    float sigma_ba;

    public float getSigma_w() {
        return sigma_w;
    }

    public void setSigma_w(float sigma_w) {
        this.sigma_w = sigma_w;
    }

    public float getSigma_bw() {
        return sigma_bw;
    }

    public void setSigma_bw(float sigma_bw) {
        this.sigma_bw = sigma_bw;
    }

    public float getSigma_a() {
        return sigma_a;
    }

    public void setSigma_a(float sigma_a) {
        this.sigma_a = sigma_a;
    }

    public float getSigma_ba() {
        return sigma_ba;
    }

    public void setSigma_ba(float sigma_ba) {
        this.sigma_ba = sigma_ba;
    }
}
