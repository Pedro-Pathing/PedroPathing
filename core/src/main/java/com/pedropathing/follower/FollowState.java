/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.follower;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathProgress;
import com.pedropathing.paths.curves.Curve;
import lombok.Getter;

public final class FollowState {
    private Pose pose = Pose.zero();
    private Velocity velocity = Velocity.zero();
    private Twist twist = Twist.zero();
    private final Path path;
    private double tangentialSpeed = 0; //Signed
    private double previousTime = System.nanoTime();
    private double deltaTime;
    private double targetHeading = 0;
    private PathProgress pathProgress;

    @Getter
    private boolean isAtParametricEnd = false;
    @Getter
    private boolean isBeforeParametricStart = true;

    public FollowState(Path path) {
        this.path = path;
    }

    public void update(Pose pose, Velocity velocity, Twist twist) {
        long nanoTime = System.nanoTime();
        deltaTime = (nanoTime - previousTime) * 1e-9;
        previousTime = nanoTime;

        this.pose = pose;
        this.velocity = velocity;
        this.twist = twist;

        pathProgress = getCurve().progressAt(pose.toVector2D());
        tangentialSpeed = velocity.toLinear().dot(pathProgress.tangent);
        targetHeading = path.interpolator.interpolate(getCurve(), pathProgress.pathCompletion); // TODO: cannot get interpolation from here

        // should be distance based because then shorter paths will work
        isBeforeParametricStart = getCurve().displacementToStart(pose.toVector2D()) <= -path.beforeStartDistance;
        isAtParametricEnd = pathProgress.distanceRemaining <= path.completionDistance;
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

    public double getDeltaTime() {
        return deltaTime;
    }

    public Curve getCurve() {
        return path.curve;
    }

    public void advanceToNextPath() {
        // probably should belong to follower but the algo needs some way of accessing this
    }

    public boolean isLastPath() {
        // probably should belong to follower but algo needs way of knowing whether it can path skip
        return false;
    }

    public boolean isFollowing() {
        return path != null;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public PathProgress getPathProgress() {
        return pathProgress;
    }
}
