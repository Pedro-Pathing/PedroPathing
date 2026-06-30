/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.follower;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.config.ConfigVar;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;
import com.pedropathing.paths.PathTracker;
import com.pedropathing.paths.curves.Curve;

import lombok.Setter;

public class Follower {
    public final Localizer localizer;
    public final Drivetrain drivetrain;
    public final ConfigVar<Boolean> holdEnd = ConfigVar.of(true);
    @Setter
    private Algorithm algorithm;
    private PathTracker pathTracker = null;
    private Pose holdPose = null;
    public DrivePowers manualPowers = null; // TODO: remove public (just for testing)
    private Mode mode = Mode.IDLE;
    private long previousNanoTime = 0L;
    private boolean useHoldScaling;

    public Follower(Localizer localizer, Drivetrain drivetrain, Algorithm algorithm) {
        this.localizer = localizer;
        this.algorithm = algorithm;
        this.drivetrain = drivetrain;
    }

    public Mode mode() {
        return mode;
    }

    public void update() {
        long nanoTime = System.nanoTime();
        double deltaTime = 0;
        if (previousNanoTime != 0L) deltaTime = (nanoTime - previousNanoTime) / 1e9;
        update(deltaTime);
        previousNanoTime = nanoTime;
    }

    public void update(double deltaTime) {
        localizer.update();

        switch (mode) {
            case FOLLOW: {
                if (pathTracker.done()) {
                    if (holdEnd.get()) {
                        hold(pathTracker.endPose());
                    } else {
                        stop();
                    }
                    pathTracker = null;
                    break;
                }

                DrivePowers powers = algorithm.calculatePath(pathTracker, localizer.state(), deltaTime);
                drivetrain.drive(powers, false);
                break;
            }
            case HOLD: {
                DrivePowers powers = algorithm.calculateHold(holdPose, localizer.state(), useHoldScaling, deltaTime);
                drivetrain.drive(powers, false);
                break;
            }
            case MANUAL: {
                drivetrain.drive(manualPowers, true);
                break;
            }
            case IDLE: {
                drivetrain.stop();
                break;
            }
        }
    }

    private void clearState() {
        if (pathTracker != null) pathTracker.release();
        pathTracker = null;
        holdPose = null;
        manualPowers = null;
        useHoldScaling = true;
    }

    public void follow(Path path) {
        clearState();
        mode = Mode.FOLLOW;
        pathTracker = new PathTracker(path);
    }

    public void hold(Pose pose) {
        clearState();
        mode = Mode.HOLD;
        holdPose = pose;
    }

    public void hold(Pose pose, boolean useScaling) {
        hold(pose);
        useHoldScaling = useScaling;
    }

    public void turn(double heading) {
        hold(pose().withHeading(heading));
    }

    public void manual(DrivePowers powers) {
        clearState();
        mode = Mode.MANUAL;
        manualPowers = powers;
    }

    public void manual(double forward, double lateral, double heading) {
        manual(new DrivePowers(forward, lateral, heading));
    }

    public void stop() {
        clearState();
        mode = Mode.IDLE;
    }

    public void setPose(Pose pose) {
        localizer.setPose(pose);
    }

    public void setX(double x) {
        localizer.setX(x);
    }

    public void setY(double y) {
        localizer.setY(y);
    }

    public void setHeading(double heading) {
        localizer.setHeading(heading);
    }

    public Pose pose() {
        return localizer.pose();
    }

    public Velocity velocity() {
        return localizer.velocity();
    }

    public Twist twist() {
        return localizer.twist();
    }

    public boolean following() {
        return mode == Mode.FOLLOW;
    }

    public boolean holding() {
        return mode == Mode.HOLD;
    }

    public boolean manual() {
        return mode == Mode.MANUAL;
    }

    public boolean idle() {
        return mode == Mode.IDLE;
    }

    public enum Mode {
        FOLLOW,
        HOLD,
        MANUAL,
        IDLE
    }

    public boolean atParametricEnd() {
        return !mode.equals(Mode.FOLLOW) || algorithm.atParametricEnd(closestT());
    }

    public double closestT() {
        return algorithm.closestT();
    }

    public Vector2D closestTangent() {
        return algorithm.closestTangent();
    }

    public Vector2D closestNormal() {
        return algorithm.closestNormal();
    }

    public double curvature() {
        return algorithm.curvature();
    }

    public Pose closestPose() {
        return algorithm.closestPose();
    }

    public double curveCompletion() {
        return algorithm.pathCompletion();
    }

    public double remainingDistance() {
        return algorithm.remainingDistance();
    }

    public int pathIndex() {
        return pathTracker.currentIndex();
    }

    public PathSegment currentSegment() {
        return pathTracker.current();
    }

    public Curve currentCurve() {
        return currentSegment().curve;
    }

    public Path currentPath() {
        return pathTracker.path();
    }

    public double getTangentialVelocity() {
        return velocity().toVector2D().dot(closestTangent());
    }
}
