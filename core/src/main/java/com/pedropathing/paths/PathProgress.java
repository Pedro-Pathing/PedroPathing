package com.pedropathing.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Vector;

public class PathProgress {
    public Pose closestPose;
    public Vector closestTangentVector;
    public double tValue;
    public double remainingDistance;
    public boolean atParametricEnd;
}