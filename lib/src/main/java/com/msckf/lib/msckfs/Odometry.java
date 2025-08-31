package com.msckf.lib.msckfs;


import com.msckf.lib.utils.Isometry3D;

import org.ejml.simple.SimpleMatrix;

public class Odometry {

    public double time;

    public Isometry3D pose; // Body pose. Usually equal to IMU pose.

    public SimpleMatrix bodyVelocity;

    public Isometry3D camPose;

    public void setTo(Odometry odom) {
        this.time = odom.time;
        this.pose = odom.pose;
        this.bodyVelocity = odom.bodyVelocity;
        this.camPose = odom.camPose;
    }

    public Odometry(double timestamp, Isometry3D pose, SimpleMatrix bodyVelocity, Isometry3D camPose) {
        this.time = timestamp;
        this.pose = pose;
        this.bodyVelocity = bodyVelocity;
        this.camPose = camPose;
    }

    @Override
    public String toString() {
        return "Odometry{" +
                "timestamp=" + time +
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
                String.valueOf(time),
                String.valueOf(pose.t.get(0)),
                String.valueOf(pose.t.get(1)),
                String.valueOf(pose.t.get(2))

        };
    }
}
