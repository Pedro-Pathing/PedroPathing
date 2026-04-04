package com.pedropathing.paths;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

public class PathProgress {
    public Pose closestPose;
    public Vector2D closestTangentVector;
    public double tValue;
    public double remainingDistance;
    public boolean atParametricEnd;
}