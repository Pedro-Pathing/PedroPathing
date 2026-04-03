package com.pedropathing.follower;

import com.pedropathing.control.controllers.Controller;
import com.pedropathing.geometry.Angle;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Matrix;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Twist;
import com.pedropathing.geometry.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathProgress;

public class BedroAlgorithm implements Algorithm {
    private final Controller headingController;
    private final Controller translationalController;
    private final Controller driveController;
    private final Matrix ellipsoidMatrix;
    private final Vector linearBraking;
    private final double centripetalScaling;
    private final double alpha;

    public BedroAlgorithm(Controller headingController, Controller translationalController, double centripetalScaling,
                          Vector eigenvalues, Vector linearBraking, double alpha, Controller driveController) {
        this.headingController = headingController;
        this.translationalController = translationalController;
        this.driveController = driveController;
        this.centripetalScaling = centripetalScaling;
        ellipsoidMatrix = new Matrix(new double[][]{
                {eigenvalues.x, 0},
                {0, eigenvalues.y}
        });
        this.linearBraking = linearBraking;
        this.alpha = alpha;
    }

    @Override
    public Twist calculate(FollowState state) {
        Vector translational = translational(state.getPose().toVector(), state.getPath().pathProgress, state.getPath().currentCurve());
        Vector centripetal = centripetal(state.getTangentialSpeed(), state.getPath().pathProgress, state.getPath().currentCurve());
        Vector drive = drive(state.getTangentialSpeed(), state.getPath().pathProgress);
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
        double curvature = curve.curvature(progress.tValue);
        Vector normal = curve.getNormal(progress.tValue);
        return normal.times(speed * speed * curvature * centripetalScaling);
    }

    public Vector drive(double tangentialVel, PathProgress progress) {
        double quadraticBrakeDirection = alpha * progress.closestTangentVector.transform(ellipsoidMatrix).dot(progress.closestTangentVector); //k2
        double linearBrakeDirection = alpha * progress.closestTangentVector.dot(linearBraking); //k1
        double targetVel = (-linearBrakeDirection + Math.sqrt(linearBrakeDirection * linearBrakeDirection
                + 4 * quadraticBrakeDirection * progress.remainingDistance)) / (2 * quadraticBrakeDirection);
        double error = targetVel - tangentialVel;
        //TODO: do we need a Kalman Filter?
        return progress.closestTangentVector.times(driveController.calculate(error));
    }
}
