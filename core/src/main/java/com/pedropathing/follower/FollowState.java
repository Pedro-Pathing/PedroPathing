package com.pedropathing.follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Twist;
import com.pedropathing.geometry.Velocity;
import com.pedropathing.paths.Path;

public final class FollowState {
    private Pose pose = Pose.zero();
    private Velocity velocity = Velocity.zero();
    private Twist twist = Twist.zero();
    private Path path;
    private double tangentialSpeed = 0; //Signed

    public FollowState(Path path) {
        this.path = path;
    }

    public void update(Pose pose, Velocity velocity, Twist twist) {
        this.pose = pose;
        this.velocity = velocity;
        this.twist = twist;
        tangentialSpeed = velocity.toLinear().dot(path.pathProgress.closestTangentVector);
    }

    public double tangentialSpeed() {
        return tangentialSpeed;
    }

    public Pose getPose() {
        return pose;
    }

    public Velocity getVelocity() {
        return velocity;
    }

    public Twist getTwist() {
        return twist;
    }

    public Path getPath() {
        return path;
    }

    public double getTangentialSpeed() {
        return tangentialSpeed;
    }
}

/*
 * Curve is BezierPoint, BezierLine, BezierCurve
 * Path(curve, curve, curve)
 * Path(path, path)
 */