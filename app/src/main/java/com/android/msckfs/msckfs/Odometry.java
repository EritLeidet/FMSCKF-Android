package com.android.msckfs.msckfs;

import androidx.annotation.NonNull;

import com.android.msckfs.utils.Isometry3D;

import org.ejml.simple.SimpleMatrix;

public class Odometry {

    public double timestamp;

    public Isometry3D pose; // Body pose. Usually equal to IMU pose.

    public SimpleMatrix bodyVelocity;

    public Isometry3D camPose;

    public void setTo(Odometry odom) {
        this.timestamp = odom.timestamp;
        this.pose = odom.pose;
        this.bodyVelocity = odom.bodyVelocity;
        this.camPose = odom.camPose;
    }

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
