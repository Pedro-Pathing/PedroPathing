package com.pedropathing.follower;

import com.pedropathing.control.controllers.Controller;
import com.pedropathing.math.Angle;
import com.pedropathing.paths.Curve;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathProgress;

public class BedroAlgorithm implements Algorithm {
    private final Controller headingController;
    private final Controller translationalController;
    private final Controller driveController;
    private final Matrix ellipsoidMatrix;
    private final Vector2D linearBraking;
    private final double centripetalScaling;
    private final double alpha;

    public BedroAlgorithm(Controller headingController, Controller translationalController, double centripetalScaling,
                          Vector2D eigenvalues, Vector2D linearBraking, double alpha, Controller driveController) {
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
    public DrivePowers calculate(FollowState state) {
        Vector2D translational = translational(state.getPose().toVector2D(), state.getPath().pathProgress, state.getPath().currentCurve());
        Vector2D centripetal = centripetal(state.getTangentialSpeed(), state.getPath().pathProgress, state.getPath().currentCurve());
        Vector2D drive = drive(state.getTangentialSpeed(), state.getPath().pathProgress);
        return new DrivePowers(0, 0, heading(state.getPose().heading, state.getPath().pathProgress.closestPose.heading));
    }

    public double heading(double current, double target) {
        return headingController.calculate(Angle.smallestDifference(current, target) * Angle.turnDirection(current, target));
    }

    public Vector2D translational(Vector2D current, PathProgress progress, Curve curve) {
        Vector2D target = progress.atParametricEnd ? curve.endPoint() : progress.closestPose.toVector2D();
        Vector2D offset = target.minus(current);
        return offset.times(translationalController.calculate(offset.magnitude()));
    }

    public Vector2D centripetal(double speed, PathProgress progress, Curve curve) {
        double curvature = curve.curvature(progress.tValue);
        Vector2D normal = curve.getNormal(progress.tValue);
        return normal.times(speed * speed * curvature * centripetalScaling);
    }

    public Vector2D drive(double tangentialVel, PathProgress progress) {
        double quadraticBrakeDirection = alpha * progress.closestTangentVector.transform(ellipsoidMatrix).dot(progress.closestTangentVector); //k2
        double linearBrakeDirection = alpha * progress.closestTangentVector.dot(linearBraking); //k1
        double targetVel = (-linearBrakeDirection + Math.sqrt(linearBrakeDirection * linearBrakeDirection
                + 4 * quadraticBrakeDirection * progress.remainingDistance)) / (2 * quadraticBrakeDirection);
        double error = targetVel - tangentialVel;
        //TODO: do we need a Kalman Filter?
        return progress.closestTangentVector.times(driveController.calculate(error));
    }
}
