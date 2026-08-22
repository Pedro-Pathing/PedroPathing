package com.pedropathing.algorithm;

import static com.pedropathing.utils.Angle.normalizeSigned;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.DiamondDrivetrainModel;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathTracker;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.utils.Pair;
import com.pedropathing.utils.Timer;
import com.pedropathing.utils.Utils;
import java.util.concurrent.TimeUnit;

public class ForesightV3 implements Algorithm {
    public final ForesightConfig config;
    public final ForesightPowerAllocator allocator;
    private double closestT, curvature;
    private double projectedClosestT;
    private double curveCompletion, remainingDistance, tangentialSpeed;
    private Pose closestPose;
    private Vector2D closestTangent, closestNormal;
    private final Timer timer = new Timer();
    private boolean resetTimer = true;
    private double headingError, translationalError, targetVelocity;
    private boolean busy = false;

    public ForesightV3(ForesightConfig config) {
        this.config = config;
        this.allocator = new ForesightPowerAllocator(config);
    }

    @Override
    public DrivePowers calculatePath(Drivetrain drivetrain, PathTracker pathTracker, MotionState state, double deltaTime) {
        closestT = pathTracker.current().curve.closestT(state.pose().toVector2D(), closestT);

        if (testParametric()) { // End Constraint
            closestT = 1.0;
            double targetHeading = pathTracker.current().heading(closestT);
            closestPose = pathTracker.current().curve.get(closestT).toPose(targetHeading);

            if (pathTracker.remainingPaths() > 1) { // advance if constraints met
                pathTracker.advance();
                reset();
                return calculatePath(drivetrain, pathTracker, state, deltaTime);
            }

            pathTracker.advance();
            reset();
            return DrivePowers.zero();
        } else {
            //TODO: Cache these instead so only computed on user request
            double targetHeading = pathTracker.current().heading(closestT);
            closestPose = pathTracker.current().curve.get(closestT).toPose(targetHeading);
            curvature = pathTracker.current().curve.curvature(closestT);
            remainingDistance = pathTracker.current().curve.remainingDistance(closestT);
            curveCompletion = 1 - remainingDistance / pathTracker.current().curve.length();
            closestTangent = pathTracker.current().curve.tangent(closestT);
            closestNormal = pathTracker.current().curve.leftNormal(closestT);
            tangentialSpeed = state.velocity().toVector2D().dot(closestTangent);
            translationalError = state.pose().distance(closestPose);
            headingError = normalizeSigned(targetHeading - state.pose().heading());
        }

        Curve curve = pathTracker.current().curve;
        Pose projectedPose = state.pose().plus(getBrakeDisplacement(state.twist(), state.pose().heading()));
        projectedClosestT = curve.closestT(projectedPose.toVector2D(), projectedClosestT);
        double targetHeading = closestPose.heading();
        double projectedTargetHeading = pathTracker.current().heading(projectedClosestT);
        Vector2D projectedTangent = curve.tangent(projectedClosestT);
        Vector2D projectedTargetPos = curve.get(projectedClosestT);
        Vector2D projectedNormal = curve.leftNormal(projectedClosestT);
        double projectedRemainingDist = curve.remainingDistance(projectedClosestT);
        if (projectedRemainingDist <= 0.01)
            projectedRemainingDist = projectedTangent.dot(projectedTargetPos.minus(projectedPose.toVector2D()));
        double angleToTangent = projectedTangent.theta() - projectedPose.heading();

        Pair<Double, Double> velocityInversion = getVelocityToBrakeInTime(projectedRemainingDist, projectedTangent, projectedPose.heading());
        double velocityToBrakeInTime = velocityInversion.first();
        double targetAcceleration = velocityInversion.second();
        boolean isBraking = velocityToBrakeInTime <= 0 || projectedRemainingDist <= 0;

        if (isBraking && (pathTracker.remainingPaths() > 1 || !config.brakeAtEnd.get())) {
            pathTracker.advance();
            return calculatePath(drivetrain, pathTracker, state, deltaTime);
        }

        double projectedHeadingPower = headingFeedback(projectedPose.heading(), projectedTargetHeading)
                - headingFeedback(state.pose().heading(), projectedTargetHeading);
        double currentHeadingPower = headingFeedback(state.pose().heading(), targetHeading);
        double headingError = normalizeSigned(projectedTargetHeading - projectedPose.heading());

        Vector2D drive = projectedTangent.times(drive(isBraking, velocityToBrakeInTime, targetAcceleration, deltaTime,
                projectedRemainingDist, angleToTangent, tangentialSpeed));

        Pair<Double, Vector2D> translationalResult = translationalCorrection(projectedPose, projectedTargetPos,
                projectedNormal);
        Vector2D translational = translationalResult.second();
        double translationalError = translationalResult.first();

        boolean atParametricStart = projectedClosestT <= config.parametricTConstraint.get();

        if (atParametricStart) {
            Vector2D displacementToStart = curve.startPoint().minus(projectedPose.toVector2D());
            double tangentDisplacementToStart = displacementToStart.dot(closestTangent);
            boolean isBeforePath =
                    tangentDisplacementToStart > 1 && translationalError > config.translationalDeviationTolerance.get();
            boolean isHeadingBeforePath = Math.abs(headingError) > config.headingDeviationTolerance.get();

            if (isBeforePath && config.cosineScale.get())
                drive = drive.times(tangentDisplacementToStart / translationalError);

            if (isHeadingBeforePath && config.cosineScale.get()) {
                if (config.turnBeforeDriving.get()) drive = Vector2D.zero();
                drive = drive.times(allocator.getDriveScalar(0, headingError));
            }
        } else {
            if (((Math.abs(headingError) > 2 * config.headingDeviationTolerance.get())
                    || (Math.abs(translationalError) > 2 * config.translationalDeviationTolerance.get()))
                    && config.cosineScale.get())
                drive = drive.times(allocator.getDriveScalar(translationalError, headingError));
        }

        return allocator.allocatePowers(
                drivetrain,
                state,
                Vector2D.zero(),
                projectedHeadingPower,
                translational,
                drive,
                currentHeadingPower,
                translationalError,
                headingError
        );
    }

    @Override
    public DrivePowers calculateHold(Drivetrain drivetrain, Pose target, MotionState state, boolean useScaling, double deltaTime) {
        if (resetTimer) {
            timer.reset();
            resetTimer = false;
        }

        closestPose = target;
        Pose projectedPose = state.pose().plus(getBrakeDisplacement(state.twist(), state.pose().heading()));
        double headingCorrection = headingFeedback(projectedPose.heading(), target.heading());
        Vector2D displacement = target.minus(projectedPose).toVector2D();

        //TODO: Cache
        translationalError = target.distance(state.pose());
        if (translationalError < 1e-9) {
            tangentialSpeed = 0;
            closestTangent = Vector2D.zero();
        } else {
            closestTangent = displacement.normalized();
            tangentialSpeed = closestTangent.dot(state.velocity().toVector2D());
        }

        if (busy && testTimeout() || (testHeading() && testTranslational() && testVelocity())) busy = false;

        Pair<Double, Vector2D> translationalResult = translationalCorrection(
                projectedPose,
                target.toVector2D(),
                closestTangent
        );
        Vector2D translational = translationalResult.second();
        translationalError = translationalResult.first();

        if (useScaling) {
            double translationalScale = config.holdPointTranslationalScaling.get();
            double headingScale = config.holdPointHeadingScaling.get();

            translational = translational.times(translationalScale);
            headingCorrection *= headingScale;
        }

        return allocator.getDrivePowers(translational, state, headingCorrection);
    }

    public Pose getBrakeDisplacement(Twist twist, double heading) {
        Vector2D linearTwist = twist.toVector2D();
        Vector2D quadratic = linearTwist.hadamardProduct(linearTwist.abs()).transform(config.quadraticBrakeCoefficients.get());
        Vector2D linear = linearTwist.transform(config.linearBrakeCoefficients.get());
        double headingDisp = twist.omega() * Math.abs(twist.omega()) * config.headingBrakeCoefficients.get().y() +
                twist.omega() * config.headingBrakeCoefficients.get().x();
        Vector2D bodyDisp = quadratic.plus(linear);
        return new Pose(0, 0, heading).exp(new Twist(bodyDisp.x(), bodyDisp.y(), headingDisp));
    }

    public Pair<Double, Double> getVelocityToBrakeInTime(double distanceRemaining, Vector2D closestTangent, double heading) {
        Vector2D t = closestTangent.toBodyFrame(heading).abs();
        Vector2D t2 = t.hadamardProduct(t);
        Vector2D t3 = t2.hadamardProduct(t);

        double k1 = config.quadraticBrakeCoefficients.get().get(0, 0) * t3.x()
                + config.quadraticBrakeCoefficients.get().get(1, 1) * t3.y();
        double k2 = config.linearBrakeCoefficients.get().get(0, 0) * t2.x()
                + config.linearBrakeCoefficients.get().get(1, 1) * t2.y();
        Pair<Double, Double> velocityInversion =
                Utils.solveQuadratic(k1, k2, -Math.abs(distanceRemaining) / config.brakeAggression.get());
        double maxVel = Math.max(velocityInversion.first(), velocityInversion.second()) * Math.signum(distanceRemaining);
        return Pair.of(maxVel, -maxVel / (2 * maxVel * k1 + k2));
    }

    private double headingFeedback(double currentHeading, double targetHeading) {
        headingError = normalizeSigned(targetHeading - currentHeading);
        return config.headingController.get().calculate(0, headingError);
    }

    public double drive(boolean isBraking,
                        double profiledTargetVelocity, double profiledTargetAcceleration,
                        double deltaTime, double remainingDistance,
                        double angleToTangent, double tangentialVel) {
        double maxAchievableVelocity = DiamondDrivetrainModel.interpolateVelocity(config.maxAchievableForwardVelocity.get(), config.maxAchievableStrafeVelocity.get(), angleToTangent);
        targetVelocity = Math.min(profiledTargetVelocity, maxAchievableVelocity);

        if (!isBraking) return coast(tangentialVel, remainingDistance, deltaTime, maxAchievableVelocity);
        return config.brakeController.get().calculate(targetVelocity, 0) + config.brakeAccelFeedforward.get().calculate(profiledTargetAcceleration, 0);
    }

    public double coast(double tangentialVel, double remainingDistance, double deltaTime, double maxAchievableVelocity) {
        double maxAccelerationConstraint = config.maxAccelerationConstraint.get();
        double maxVelocityConstraint = config.maxVelocityConstraint.get();
        double maxDecelerationConstraint = config.maxDecelerationConstraint.get();
        double coastDownToVelocity = config.coastDownToVelocity.get();

        double targetVel = maxAchievableVelocity;

        if (maxVelocityConstraint != ForesightConfig.Constraint.NONE)
            targetVel = Math.min(targetVel, maxVelocityConstraint);

        if (maxAccelerationConstraint != ForesightConfig.Constraint.NONE)
            targetVel = Math.min(targetVel, tangentialVel + maxAccelerationConstraint * deltaTime);

        double feedforwardVelocity = targetVel;

        if (maxDecelerationConstraint != ForesightConfig.Constraint.NONE) {
            double velocityNeededToCoastInTime = Math.sqrt(coastDownToVelocity * coastDownToVelocity
                    + 2 * maxDecelerationConstraint * remainingDistance);
            targetVel = Math.min(targetVel, velocityNeededToCoastInTime);
        }

        if (targetVel >= maxAchievableVelocity) {
            targetVelocity = maxAchievableVelocity;
            return 1;
        }

        double error = Math.max(0, targetVel);
        targetVelocity = targetVel;
        return config.coastController.get().calculate(feedforwardVelocity, error);
    }

    private Pair<Double, Vector2D> translationalCorrection(Pose currentPose, Vector2D targetPos, Vector2D closestNormal) {
        Vector2D displacement = targetPos.minus(currentPose.toVector2D()).projectOnto(closestNormal);
        double translationalError = displacement.magnitude();
        if (translationalError < config.minCorrectionDistance.get()) return Pair.of(0.0, Vector2D.zero());

        Vector2D bodyFrameError = displacement.toBodyFrame(currentPose.heading());
        return Pair.of(translationalError, Vector2D.cartesian(
                config.forwardTranslationalController.get().calculate(0, bodyFrameError.x()),
                config.lateralTranslationalController.get().calculate(0, bodyFrameError.y())
        ).projectOnto(bodyFrameError).toWorldFrame(currentPose.heading()));
    }

    public double getHeadingError() {
        return headingError;
    }

    public double getTranslationalError() {
        return translationalError;
    }

    public boolean testVelocity() {
        return tangentialSpeed < config.velocityConstraint.get();
    }

    public boolean testTranslational() {
        return Math.abs(translationalError) < config.translationalConstraint.get();
    }

    public boolean testHeading() {
        return Math.abs(headingError) < config.headingConstraint.get();
    }

    public boolean testParametric() {
        return closestT >= (1 - config.parametricTConstraint.get());
    }

    public boolean testTimeout() {
        return !resetTimer && timer.get(TimeUnit.MILLISECONDS) > config.timeoutConstraint.get();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    @Override
    public double closestT() {
        return closestT;
    }

    @Override
    public Pose closestPose() {
        return closestPose;
    }

    @Override
    public Vector2D closestTangent() {
        return closestTangent;
    }

    @Override
    public Vector2D closestNormal() {
        return closestNormal;
    }

    @Override
    public double curvature() {
        return curvature;
    }

    @Override
    public double remainingDistance() {
        return remainingDistance;
    }

    @Override
    public double pathCompletion() {
        return curveCompletion;
    }

    @Override
    public boolean atParametricEnd(double t) {
        return testParametric();
    }

    @Override
    public void reset() {
        timer.reset();
        resetTimer = true;
        busy = true;
        closestT = 0.0;
    }

    @Override
    public boolean isBusy() {
        return busy;
    }
}
