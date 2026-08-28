package com.pedropathing.algorithm;

import static com.pedropathing.utils.Angle.normalizeSigned;
import static com.pedropathing.utils.Angle.turnDirection;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.DiamondDrivetrainModel;
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
    private double projectedClosestT, coastClosestT;
    private double curveCompletion, remainingDistance, tangentialSpeed;
    private Pose closestPose;
    private Vector2D closestTangent, closestNormal;
    private final Timer timer = new Timer();
    private boolean resetTimer = true;
    private double headingError, translationalError, targetVelocity;
    private boolean busy = false;
    private final Vector2D naturalDeceleration;

    public ForesightV3(ForesightConfig config) {
        this.config = config;
        this.allocator = new ForesightPowerAllocator(config);
        this.naturalDeceleration = Vector2D.cartesian(config.naturalForwardDeceleration.get(),
                config.naturalStrafeDeceleration.get());
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
            this.headingError = normalizeSigned(targetHeading - state.pose().heading());
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

        double velocityToBrakeInTime = getVelocityToBrakeInTime(projectedRemainingDist, projectedTangent, projectedPose.heading());
        boolean isBraking = velocityToBrakeInTime <= 0 || projectedRemainingDist <= 0;

        if (isBraking && (pathTracker.remainingPaths() > 1 || !config.brakeAtEnd.get())) {
            pathTracker.advance();
            return calculatePath(drivetrain, pathTracker, state, deltaTime);
        }

        double headingError = normalizeSigned(projectedTargetHeading - projectedPose.heading());
        double currentHeadingError = normalizeSigned(targetHeading - state.pose().heading());

        double totalHeadingPower = config.headingFeedback.get().calculate(0, headingError);
        double headingFeedbackPower = config.headingFeedback.get().calculate(0, currentHeadingError);
        double headingFeedforwardPower;

        if (Math.abs(totalHeadingPower) <= 1e-3) {
            headingFeedbackPower = 0;
            headingFeedforwardPower = 0;
        } else if (totalHeadingPower * headingFeedbackPower < 0) {
            headingFeedforwardPower = totalHeadingPower;
            headingFeedbackPower = 0;
        } else if (Math.abs(headingFeedbackPower) >= Math.abs(totalHeadingPower)) {
            headingFeedforwardPower = 0;
            headingFeedbackPower = totalHeadingPower;
        } else {
            headingFeedforwardPower = totalHeadingPower - headingFeedbackPower;
        }

        headingFeedforwardPower += config.headingStaticFF.get().calculate(0, turnDirection(headingError));

        Vector2D drive = projectedTangent.times(drive(isBraking, velocityToBrakeInTime, deltaTime,
                projectedRemainingDist, angleToTangent, tangentialSpeed, state, curve));

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
                headingFeedforwardPower,
                translational,
                drive,
                headingFeedbackPower,
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
        double headingCorrection = headingFeedback(projectedPose.heading(), target.heading()).second();
        headingError = normalizeSigned(target.heading() - state.pose().heading());

        Vector2D displacement = target.minus(projectedPose).toVector2D();
        double dist = displacement.magnitude();

        translationalError = target.distance(state.pose());
        if (dist < 1e-9) {
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
                displacement.div(dist)
        );
        Vector2D translational = translationalResult.second();

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
        double headingDisp = twist.omega * Math.abs(twist.omega) * config.headingBrakeCoefficients.get().y() +
                twist.omega * config.headingBrakeCoefficients.get().x();
        Vector2D bodyDisp = quadratic.plus(linear);
        Pose worldPose = new Pose(0, 0, heading).exp(new Twist(bodyDisp.x(), bodyDisp.y(), headingDisp));
        return new Pose(worldPose.x(), worldPose.y(), headingDisp);
    }

    public Pose getCoastDisplacement(Twist twist, double heading) {
        Vector2D linearTwist = twist.toVector2D();
        Vector2D bodyDisp = linearTwist.hadamardProduct(linearTwist.abs())
                .times(1.0 / 2.0)
                .elementDivision(naturalDeceleration);
        double headingDisp = twist.omega * Math.abs(twist.omega) * config.headingBrakeCoefficients.get().y() +
                twist.omega * config.headingBrakeCoefficients.get().x();
        Pose worldPose = new Pose(0, 0, heading).exp(new Twist(bodyDisp.x(), bodyDisp.y(), headingDisp));
        return new Pose(worldPose.x(), worldPose.y(), headingDisp);
    }

    public double getVelocityToBrakeInTime(double distanceRemaining, Vector2D closestTangent, double heading) {
        Vector2D t = closestTangent.toBodyFrame(heading).abs();
        Vector2D t2 = t.hadamardProduct(t);
        Vector2D t3 = t2.hadamardProduct(t);

        double k1 = config.quadraticBrakeCoefficients.get().get(0, 0) * t3.x()
                + config.quadraticBrakeCoefficients.get().get(1, 1) * t3.y();
        double k2 = config.linearBrakeCoefficients.get().get(0, 0) * t2.x()
                + config.linearBrakeCoefficients.get().get(1, 1) * t2.y();
        Pair<Double, Double> velocityInversion =
                Utils.solveQuadratic(k1, k2, -Math.abs(distanceRemaining) / config.brakeAggression.get());
        return Math.max(velocityInversion.first(), velocityInversion.second()) * Math.signum(distanceRemaining);
    }

    private Pair<Double, Double> headingFeedback(double currentHeading, double targetHeading) {
        double headingError = normalizeSigned(targetHeading - currentHeading);
        return Pair.of(headingError, config.headingFeedback.get().calculate(0, headingError)
                + (config.headingStaticFF.get().calculate(0, turnDirection(headingError))));
    }

    public double drive(boolean isBraking,
                        double profiledTargetVelocity,
                        double deltaTime, double remainingDistance,
                        double angleToTangent, double tangentialVel,
                        MotionState state, Curve curve) {
        double maxAchievableVelocity = DiamondDrivetrainModel.interpolateVelocity(config.maxAchievableForwardVelocity.get(), config.maxAchievableStrafeVelocity.get(), angleToTangent);
        targetVelocity = Math.min(profiledTargetVelocity, maxAchievableVelocity);

        if (!isBraking) return coast(tangentialVel, deltaTime, maxAchievableVelocity, state, curve, angleToTangent);
        return config.brake.get().calculate(targetVelocity, 0);
    }

    public double coast(double tangentialVel, double deltaTime, double maxAchievableVelocity,
                        MotionState state, Curve curve, double theta) {
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
            Pose projected = state.pose().plus(getCoastDisplacement(state.twist(), state.pose().heading()));
            double projectedT = curve.closestT(projected.toVector2D(), coastClosestT);
            Vector2D projectedTangent = curve.tangent(projectedT);
            double projectedRemainingDist = curve.remainingDistance(projectedT);
            if (projectedRemainingDist <= 0.01)
                projectedRemainingDist = projectedTangent.dot(curve.get(projectedT).minus(projected.toVector2D()));
            double discrim = coastDownToVelocity * coastDownToVelocity + 2 * maxDecelerationConstraint * projectedRemainingDist;
            double excessVel = excessVelAfterCoast(projectedRemainingDist, theta);
            double velocityMomentumCannotProvide = Math.max(0, coastDownToVelocity - excessVel);
            double velocityNeededToCoastInTime = Math.sqrt(Math.abs(discrim)) * Math.signum(discrim);
            targetVel = Math.min(targetVel, velocityNeededToCoastInTime);
            feedforwardVelocity = Math.min(feedforwardVelocity, velocityMomentumCannotProvide);
        }

        if (targetVel >= maxAchievableVelocity) {
            targetVelocity = maxAchievableVelocity;
            return 1;
        }

        double error = Math.max(0, targetVel);
        targetVelocity = targetVel;
        return Math.max(config.coast.get().calculate(feedforwardVelocity, error), 0);
    }

    private Pair<Double, Vector2D> translationalCorrection(Pose currentPose, Vector2D targetPos, Vector2D closestNormal) {
        Vector2D displacement = targetPos.minus(currentPose.toVector2D()).projectOnto(closestNormal);
        double translationalError = displacement.magnitude();
        if (translationalError < config.minCorrectionDistance.get()) return Pair.of(0.0, Vector2D.zero());

        Vector2D bodyFrameError = displacement.toBodyFrame(currentPose.heading());
        return Pair.of(translationalError, Vector2D.cartesian(
                config.forwardTranslational.get().calculate(0, bodyFrameError.x()),
                config.strafeTranslational.get().calculate(0, bodyFrameError.y())
        ).projectOnto(bodyFrameError).toWorldFrame(currentPose.heading()));
    }

    private double excessVelAfterCoast(double remainingDistance, double theta) {
        double naturalDeceleration = DiamondDrivetrainModel.interpolateAcceleration(config.naturalForwardDeceleration.get(), config.naturalStrafeDeceleration.get(), theta);
        double excessVelocitySquared = -2 * naturalDeceleration * remainingDistance;
        return Math.signum(excessVelocitySquared) * Math.sqrt(Math.abs(excessVelocitySquared));
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
        projectedClosestT = 0.0;
        coastClosestT = 0.0;
    }

    @Override
    public boolean isBusy() {
        return busy;
    }
}
