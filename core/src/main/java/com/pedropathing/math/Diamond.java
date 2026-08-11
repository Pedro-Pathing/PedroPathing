package com.pedropathing.math;

public class Diamond {
    public static double interpolateRadius(double xRadius, double yRadius, double theta) {
        return 1.0 / (Math.abs(Math.cos(theta)) / xRadius + Math.abs(Math.sin(theta)) / yRadius);
    }
}