package com.pedropathing.paths;

import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.curves.Curve;


public class PathProgress {
    public final Vector2D point;
    public final Vector2D tangent;
    public final Vector2D normal;
    public final double pathCompletion;
    public final double distanceRemaining;
    public final double curvature;

    public PathProgress(Vector2D point, Vector2D tangent, double remainingDistance, double pathCompletion, double curvature) {
        this.point = point;
        this.tangent = tangent;
        this.normal = tangent.perpendicularLeft();
        this.distanceRemaining = remainingDistance;
        this.pathCompletion = pathCompletion;
        this.curvature = curvature;
    }

    public static PathProgress at(Curve curve, double tValue) {
        return new PathProgress(
                curve.get(tValue),
                curve.tangent(tValue),
                curve.distanceRemaining(tValue),
                tValue,
                curve.curvature(tValue)
        );
    }
}