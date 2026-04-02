package com.pedropathing.follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Twist;
import com.pedropathing.geometry.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathProgress;

public final class FollowState {
    public final Pose pose;
    public final Velocity velocity;
    public final Twist twist;
    public final Path path;
    public final PathProgress pathProgress;
    public FollowState(Pose pose, Velocity velocity, Twist twist, Path path, PathProgress pathProgress) {
        this.pose = pose;
        this.velocity = velocity;
        this.twist = twist;
        this.path = path;
        this.pathProgress = pathProgress;
    }
}

/*
 * Curve is BezierPoint, BezierLine, BezierCurve
 * Path(curve, curve, curve)
 * Path(path, path)
 */