// package com.pedropathing.algorithm;
//
// import com.pedropathing.controllers.Controller;
// import com.pedropathing.drivetrain.DrivePowers;
// import com.pedropathing.follower.FollowState;
// import com.pedropathing.math.Angles;
// import com.pedropathing.math.Ellipse2D;
// import com.pedropathing.math.MathFunctions;
// import com.pedropathing.math.Matrix;
// import com.pedropathing.math.Pose;
// import com.pedropathing.math.Twist;
// import com.pedropathing.math.Vector2D;
// import com.pedropathing.math.Velocity;
// import com.pedropathing.paths.curves.Curve;
// import com.pedropathing.paths.PathProgress;
// import com.pedropathing.utils.Pair;
//
// import java.util.Optional;
//
// public class IMCVCAlgorithm implements Algorithm {
//    private final Controller headingController;
//    private final Controller translationalController;
//    private final Controller brakeController;
//    private final Controller coastController;
//    private final Matrix quadraticBrake;
//    private final Matrix linearBrake;
//
//    private final Ellipse2D maxAchievableVelocity;
//    private final Ellipse2D coastingDecelerationConstraint;
//    private final double maxAccelerationConstraint;
//    private final double maxVelocityConstraint;
//    private final double velocityToCoastToBeforeBraking;
//
//    private final double centripetalScaling;
//    private final double alpha; //default=1.0
//
//    public IMCVCAlgorithm(Controller headingController, Controller translationalController, Controller
// coastController, double centripetalScaling,
//                          Vector2D quadraticBrakeVals, Vector2D linearBrakeVals, double alpha, Controller
// brakeController, double forwardMaxVel,
//                          double lateralMaxVel, Optional<Double> maxVelocityConstraint,
//                          double forwardZPA, double lateralZPA,
//                          Optional<Double> coastStrength,
//                          Optional<Double> maxAccelerationConstraint,
//                          Optional<Double> velocityToCoastToBeforeBraking) {
//        this.headingController = headingController;
//        this.translationalController = translationalController;
//        this.coastController = coastController;
//        this.brakeController = brakeController;
//        this.centripetalScaling = centripetalScaling;
//        quadraticBrake = Matrix.diag(quadraticBrakeVals.x, quadraticBrakeVals.y);
//        linearBrake = Matrix.diag(linearBrakeVals.x, linearBrakeVals.y);
//        this.alpha = alpha;
//        maxAchievableVelocity = Ellipse2D.fromAxes(forwardMaxVel, lateralMaxVel);
//
//        coastingDecelerationConstraint =
//            coastStrength.map(aDouble -> Ellipse2D.fromAxes(forwardZPA * 4 * aDouble,
//                                                            lateralZPA * 4 * aDouble))
//                .orElseGet(() -> Ellipse2D.fromAxes(Double.POSITIVE_INFINITY,
//                                                    Double.POSITIVE_INFINITY));
//        this.maxVelocityConstraint =
//            maxVelocityConstraint.orElse(Double.POSITIVE_INFINITY);
//        this.maxAccelerationConstraint = maxAccelerationConstraint.orElse(Double.POSITIVE_INFINITY);
//        this.velocityToCoastToBeforeBraking = velocityToCoastToBeforeBraking.orElse(0.0);
//    }
//
//    @Override
//    public DrivePowers calculate(FollowState state) {
//        Vector2D translational = translational(state.getPose(), state.getVelocity(), state.getPath().pathProgress,
// state.getPath().currentCurve());
//        Vector2D centripetal = centripetal(state.getTangentialSpeed(), state.getPath().pathProgress,
// state.getPath().currentCurve());
//        Vector2D drive = drive(state.getTangentialSpeed(), state.getPath().pathProgress
//            , state.getPose().heading, state.getDeltaTime());
//        return new DrivePowers(0, 0, heading(state.getPose().heading,
// state.getPath().pathProgress.closestPose.heading));
//    }
//
//    public double heading(double current, double target) {
//        return headingController.calculate(target, Angles.smallestDifference(current, target) *
// Angles.turnDirection(current, target));
//    }
//
//    public Vector2D translational(Pose currentPose, Velocity velocity, PathProgress progress, Curve curve) {
//        if (progress.atParametricStart) {
//            Vector2D displacement = progress.closestPose.minus(currentPose).toVector2D();
//            return computeTranslationalCorrection(displacement, velocity, currentPose.heading);
//        }
//
//        if (progress.atParametricEnd) {
//            Vector2D displacement = curve.endPoint().minus(currentPose.toVector2D());
//            return computeTranslationalCorrection(displacement, velocity, currentPose.heading);
//        }
//
//        double error = currentPose.distance(progress.closestPose);
//        Vector2D gradientError = curve.leftNormal(progress.tValue).times(error);
//        return computeTranslationalCorrection(gradientError, velocity, currentPose.heading);
//    }
//
//    private Vector2D computeTranslationalCorrection(Vector2D displacementVector, Velocity velocity, double
// currentHeading) {
//        Vector2D linearVel = velocity.toLinear().projectOnto(displacementVector);
//        double theta = linearVel.angleTo(Vector2D.unit(currentHeading));
//        Vector2D adjustedError = displacementVector.minus(getBrakeDisplacement(linearVel.magnitude(), theta,
// Math.signum(linearVel.dot(displacementVector)))
//                .toVelocity(currentHeading).toLinear());
//        double distance = adjustedError.magnitude();
//        if (distance < 1e-3) return Vector2D.zero(); //TODO: Scale 1e-3 according to translational constraint?
//        return adjustedError.times(translationalController.calculate(0, distance)).div(distance);
//    }
//
//    public Vector2D centripetal(double speed, PathProgress progress, Curve curve) {
//        double curvature = curve.curvature(progress.tValue);
//        Vector2D normal = curve.leftNormal(progress.tValue);
//        if (normal.isZero()) return Vector2D.zero();
//        return normal.times(speed * speed * curvature * centripetalScaling);
//    }
//
//    public Vector2D drive(double tangentialVel, PathProgress progress, double heading,
//                          double deltaTime) {
//        double maxVelocityToFitAccel =
//            tangentialVel + maxAccelerationConstraint * deltaTime;
//        double constrainedVelocity = Math.min(maxVelocityConstraint, maxVelocityToFitAccel);
//        double currentMaxAchievableVelocity =
//            maxAchievableVelocity.radius(Vector2D.unit(heading).angleTo(progress.closestTangentVector));
//        constrainedVelocity = Math.min(constrainedVelocity, currentMaxAchievableVelocity);
//
//        double theta = progress.closestTangentVector.angleTo(Vector2D.unit(heading));
//
//        double cos = Math.cos(theta);
//        double sin = Math.sin(theta);
//        double k1 = quadraticBrake.get(0, 0) * cos * cos * cos + quadraticBrake.get(1, 1) * sin * sin * sin;
//        double k2 = linearBrake.get(0, 0) * cos * cos + linearBrake.get(1, 1) * sin * sin;
//        Pair<Double, Double> velocityInversion = MathFunctions.solveQuadratic(k1, k2,
// -progress.remainingDistance/alpha);
//        double targetVelocityToBrakeInTime = Math.max(velocityInversion.first(), velocityInversion.second());
//
//        boolean isBraking = tangentialVel >= targetVelocityToBrakeInTime;
//        if (!isBraking) {
//            double targetCoastDecel = coastingDecelerationConstraint.radius(theta);
//            double coastTargetVel =
//                Math.sqrt(velocityToCoastToBeforeBraking * velocityToCoastToBeforeBraking + 2 *
// Math.abs(targetCoastDecel) * progress.remainingDistance);
//            double targetVel = Math.min(coastTargetVel, constrainedVelocity);
//            double error = Math.max(0, targetVel - tangentialVel);
//            return progress.closestTangentVector.times(coastController.calculate(targetVel, error));
//        }
//
//        double targetVel = Math.min(targetVelocityToBrakeInTime, constrainedVelocity);
//        double error = targetVel - tangentialVel;
//
//        //TODO: do we need a Kalman Filter?
//        return progress.closestTangentVector.times(brakeController.calculate(targetVel, error));
//    }
//
//    private Twist getBrakeDisplacement(double v, double theta, double sign) {
//        Vector2D unit = Vector2D.unit(theta);
//        Vector2D quadraticTerm = unit.hadamardProduct(unit).transform(quadraticBrake).times(v * v);
//        Vector2D linearTerm = unit.transform(linearBrake).times(Math.abs(v));
//        return Twist.fromVector(quadraticTerm.plus(linearTerm).times(sign));
//    }
// }
