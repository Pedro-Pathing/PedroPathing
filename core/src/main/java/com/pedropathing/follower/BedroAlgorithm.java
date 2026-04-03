package com.pedropathing.follower;

import com.pedropathing.control.controllers.Controller;
import com.pedropathing.geometry.Angle;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Twist;
import com.pedropathing.geometry.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathProgress;

public class BedroAlgorithm implements Algorithm {
    private final Controller headingController;
    private final Controller translationalController;
    private final double centripetalScaling;

    public BedroAlgorithm(Controller headingController, Controller translationalController, double centripetalScaling) {
        this.headingController = headingController;
        this.translationalController = translationalController;
        this.centripetalScaling = centripetalScaling;
    }

    @Override
    public Twist calculate(FollowState state) {
        Vector translational = translational(state.getPose().toVector(), state.getPath().pathProgress, state.getPath().currentCurve());
        Vector centripetal = centripetal(state.tangentialSpeed(), state.getPath().pathProgress, state.getPath().currentCurve());
        return new Twist(0, 0, heading(state.getPose().heading, state.getPath().pathProgress.closestPose.heading));
    }

    public double heading(double current, double target) {
        return headingController.calculate(Angle.smallestDifference(current, target) * Angle.turnDirection(current, target));
    }

    public Vector translational(Vector current, PathProgress progress, Curve curve) {
        Vector target = progress.atParametricEnd ? curve.endPoint() : progress.closestPose.toVector();
        Vector offset = target.minus(current);
        return offset.times(translationalController.calculate(offset.magnitude()));
    }

    public Vector centripetal(double speed, PathProgress progress, Curve curve) {
        if (progress.atParametricEnd) return Vector.zero();
        double curvature = curve.curvature(progress.tValue);
        Vector normal = curve.getNormal(progress.tValue);
        return normal.times(speed * speed * curvature * centripetalScaling);
    }
}
