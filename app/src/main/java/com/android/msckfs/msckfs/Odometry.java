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

    public static String[] getHeadline() {
        return new String[]{
                "t[s:double]",
                "p.x[m:double]",
                "p.y[m:double]",
                "p.z[m:double]"
        };
    }
    public String[] toStringArray() {
        return new String[]{
                String.valueOf(timestamp),
                String.valueOf(pose.t.get(0)),
                String.valueOf(pose.t.get(1)),
                String.valueOf(pose.t.get(2))

        };
    }
}
