package com.android.msckfs.msckfs;

import androidx.annotation.NonNull;

import com.android.msckfs.utils.Isometry3D;

import org.ejml.simple.SimpleMatrix;

public class Odometry {

    public final double timestamp;

    public final Isometry3D pose; // Body pose. Usually equal to IMU pose.

    public final SimpleMatrix bodyVelocity;

    public final Isometry3D camPose;

    public Odometry(double timestamp, Isometry3D pose, SimpleMatrix bodyVelocity, Isometry3D camPose) {
        this.timestamp = timestamp;
        this.pose = pose;
        this.bodyVelocity = bodyVelocity;
        this.camPose = camPose;
    }

    @Override
    public String toString() {
        return "Odometry{" +
                "timestamp=" + timestamp +
                ", pose=" + pose +
                ", bodyVelocity=" + bodyVelocity +
                ", camPose=" + camPose +
                '}';
    }
}
