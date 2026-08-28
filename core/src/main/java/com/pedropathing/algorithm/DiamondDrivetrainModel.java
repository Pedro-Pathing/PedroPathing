package com.pedropathing.algorithm;

public class DiamondDrivetrainModel {
    public static double interpolateVelocity(double xRadius, double yRadius, double theta) {
        return 1.0 / (Math.abs(Math.cos(theta)) / xRadius + Math.abs(Math.sin(theta)) / yRadius);
    }

    public static double interpolateAcceleration(double xRadius, double yRadius, double theta) {
        double cos = Math.abs(Math.cos(theta));
        double cos3 = cos * cos * cos;
        double sin = Math.abs(Math.sin(theta));
        double sin3 = sin * sin * sin;
        return 1.0 / (Math.abs(cos3) / xRadius + Math.abs(sin3) / yRadius);
    }
}