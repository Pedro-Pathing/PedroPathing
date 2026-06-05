/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.algorithm;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.FollowState;
import com.pedropathing.math.Ellipse2D;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Pair;
import com.pedropathing.utils.Utils;
import com.pedropathing.utils.Utils.Control;

import static com.pedropathing.utils.Utils.Angle.normalizeSigned;

public class Foresight implements Algorithm {
    private final ForesightConfig config;
    private final Ellipse2D maxAchievableVelocity, maxAchievableDeceleration;

    public Foresight(ForesightConfig config) {
        this.config = config;

        maxAchievableVelocity =
                Ellipse2D.fromAxes(config.maxAchievableForwardVelocity.get(), config.maxAchievableStrafeVelocity.get());
        maxAchievableDeceleration = Ellipse2D.fromAxes(
                config.maxAchievableForwardDeceleration.get(), config.maxAchievableStrafeDeceleration.get());
    }

    @Override
    public DrivePowers calculate(FollowState state) {
        double t = state.pathTracker()
                .current()
                .closestT(state.motionState().pose().toVector2D());
        double targetHeading = state.pathTracker().current().heading(t);
        double translationalError;
        Vector2D driveVector, translationalVector;

        if (t >= (1 - config.parametricTConstraint.get())) { // End Constraint
            if (state.pathTracker().size() > 1) { // advance if constraints met
                state.pathTracker().advance();
                return calculate(state);
            }

            state.pathTracker().advance();
            state.pathTracker().isFollowing(false);
            return DrivePowers.zero();
        }

        double headingError = headingError(state.motionState().pose().heading(), targetHeading);
        double headingPower = headingPower(state, targetHeading);
        double remainingDistance = state.pathTracker().current().remainingDistance(t);

        // Compute tangent and normal first so braking can consider the angle between
        // the path tangent and the robot heading (theta) instead of using heading alone.
        Vector2D closestTangentVector = state.pathTracker().current().tangent(t);
        Vector2D closestNormalVector = state.pathTracker().current().leftNormal(t);
        double thetaForBraking = closestTangentVector.angleTo(Vector2D.unit(state.motionState().pose().heading()));
        double velocityToBrakeInTime = getVelocityToBrakeInTime(remainingDistance, thetaForBraking);
        double tangentialSpeed =
                closestTangentVector.dot(state.motionState().velocity().toLinear());
        boolean isBraking = tangentialSpeed >= velocityToBrakeInTime;
        // may want hard switch? or maybe add some hysteresis?
        // or hard switch until velocity is going to change directions if it continues to brake?

        boolean pathSkip = isBraking && (state.pathTracker().size() > 1 || !config.brakeAtEnd.get());

        if (pathSkip) {
            state.pathTracker().advance();
            return calculate(state);
        }

        Vector2D brakingDisplacement = getBrakeDisplacement(state.motionState().twist(), state.motionState().pose().heading());

        driveVector = closestTangentVector.times(drive(
                tangentialSpeed,
                closestTangentVector,
                state.motionState().pose().heading(),
                state.deltaTime(),
                velocityToBrakeInTime,
                isBraking,
                remainingDistance,
                brakingDisplacement.dot(closestTangentVector))
        );

        if (t <= config.parametricTConstraint.get()) { // Start Constraint
            Vector2D start = state.pathTracker().current().startPoint();
            Vector2D displacementToStart = start.minus(state.motionState().pose().toVector2D());
            translationalError = displacementToStart.magnitude();
            double tangentDisplacementToStart = displacementToStart.dot(state.pathTracker().current().tangent(t));
            boolean isBeforePathStart = tangentDisplacementToStart > 1e-3; // originally was 0.0 for t

            // If the start point lies ahead of the robot along the path tangent (dot > 0)
            // we need to apply a translational correction to drive toward the start.
            if (isBeforePathStart) {
                translationalVector = computeTranslationalCorrection(
                                displacementToStart,
                                brakingDisplacement
                );
                driveVector = driveVector.times(tangentDisplacementToStart / translationalError);
                return allocatePowers(
                        state,
                        translationalVector,
                        driveVector,
                        headingPower,
                        translationalError,
                        headingError);
            }
        }

        translationalError = translationalError(
                state.motionState().pose(), state.pathTracker().current().get(t), closestNormalVector);
        double translationalPower = computeTranslationalCorrection(
                        closestNormalVector.times(translationalError),
                        brakingDisplacement)
                .dot(closestNormalVector);
        double centripetal = centripetal(tangentialSpeed, state.pathTracker().current().curvature(t));
        translationalPower = translationalPower + centripetal;
        translationalVector = closestNormalVector.times(translationalPower);

        if ((Math.abs(headingError) > 2 * config.headingDeviationTolerance.get()) || (Math.abs(translationalError) > 2 * config.translationalDeviationTolerance.get()))
            driveVector = driveVector.times(getDriveScalar(translationalError, headingError));

        return allocatePowers(
                state,
                translationalVector,
                driveVector,
                headingPower,
                translationalError,
                headingError);
    }

    @Override
    public DrivePowers hold(Pose target, FollowState state) {
        Vector2D translationalError = target.minus(state.motionState().pose()).toVector2D();
        Vector2D translational = computeTranslationalCorrection(
                translationalError,
                getBrakeDisplacement(state.motionState().twist(), state.motionState().pose().heading()));
        double headingPower = headingPower(state, target.heading());
        // Apply hold-point scalers. Clamp the scaler values to [0, 1] at runtime to
        // avoid accidental amplification if the configuration is set incorrectly.
        double translationalScale = config.holdPointTranslationalScaling.get();
        double headingScale = config.holdPointHeadingScaling.get();

        translational = translational.times(translationalScale);
        headingPower *= headingScale;
        return getDrivePowers(translational, state, headingPower);
    }

    /**
     * Compute heading correction power for the given state and target heading.
     */
    public double headingPower(FollowState state, double targetHeading) {
        double current = state.motionState().pose().heading();
        double error = -headingError(current, targetHeading);
        return config.headingController.get().calculate(0, error);
    }

    /**
     * Gives a drive scalar to scale down the drive power based on the translational and
     * heading errors. This is to prevent aggressive drive correction when the robot
     * is deviating a lot from the path or facing the wrong direction.
     */
    public double getDriveScalar(double normalError, double headingError) {
        double trackDeviationScale = Control.cosineScale(normalError, config.translationalDeviationTolerance.get());
        double headingScale = Control.cosineScale(headingError, config.headingDeviationTolerance.get());
        return trackDeviationScale * headingScale;
    }

    private static final int TRANSLATIONAL = 0;
    private static final int HEADING = 1;
    private static final int DRIVE = 2;

    public DrivePowers allocatePowers(FollowState state, Vector2D translationalVector, Vector2D driveVector, double headingPower, double translationalError, double headingError) {
        boolean translationalPriority = Math.abs(translationalError) > config.translationalDeviationTolerance.get();
        boolean headingPriority = Math.abs(headingError) > config.headingDeviationTolerance.get();
        double translationalPower = translationalVector.magnitude();
        double drivePower = driveVector.magnitude();

        int[] prioritization;
        double[] powers;

        if (translationalPriority && headingPriority) {
            prioritization = new int[] {0, 1, 2};
            powers = new double[] {translationalPower, headingPower, drivePower};
        } else if (translationalPriority) {
            prioritization = new int[] {0, 2, 1};
            powers = new double[] {translationalPower, drivePower, headingPower};
        } else {
            prioritization = new int[] {1, 2, 0};
            powers = new double[] {drivePower, translationalPower, headingPower};
        }

        powers = clampPowers(powers);

        Vector2D translationalDirection = Math.abs(translationalPower) < 1e-6 ? Vector2D.zero() : translationalVector.div(translationalPower);
        Vector2D driveDirection = Math.abs(drivePower) < 1e-6 ? Vector2D.zero() : driveVector.div(drivePower);

        Vector2D fieldRelativeDrivePower = translationalDirection
                .times(powers[prioritization[TRANSLATIONAL]])
                .plus(driveDirection.times(powers[prioritization[DRIVE]]));

        return getDrivePowers(fieldRelativeDrivePower, state, powers[prioritization[HEADING]]);
    }

    private double[] clampPowers(double[] powers) {
        double magnitudeRemaining = 1.0;
        double[] usedPowers = new double[3];

        for (int i = 0; i < usedPowers.length; i++) {
            double used = Control.allocatePower(powers[i], magnitudeRemaining);
            magnitudeRemaining = Control.getRemainingMagnitude(magnitudeRemaining, used);
            usedPowers[i] = used;
        }

        return usedPowers;
    }

    public DrivePowers getDrivePowers(Vector2D fieldRelativeDrivePower, FollowState state, double headingPower) {
        Vector2D robotFrameDrivePower = fieldRelativeDrivePower.rotate(-state.motionState().pose().heading());
        double forward = Control.clampBrakingPower(robotFrameDrivePower.x(), state.motionState().twist().vx(), config.maxBrakingPower.get());
        double strafe = Control.clampBrakingPower(robotFrameDrivePower.y(), state.motionState().twist().vy(), config.maxBrakingPower.get());
        return new DrivePowers(forward, -strafe, headingPower);
    }

    public double headingError(double current, double target) {
        return normalizeSigned(target - current);
    }

    public double translationalError(Pose currentPose, Vector2D closestPointVector, Vector2D closestNormalVector) {
        return currentPose.toVector2D().minus(closestPointVector).dot(closestNormalVector);
    }

    public Vector2D computeTranslationalCorrection(Vector2D displacementVector, Vector2D brakingDisplacement) {
        if (displacementVector == null || displacementVector.isZero()) return Vector2D.zero();
        Vector2D adjustedError = displacementVector.minus(brakingDisplacement);
        double distance = adjustedError.magnitude();
        if (distance < config.minCorrectionDistance.get()) return Vector2D.zero();
        return adjustedError
                .times(config.translationalController.get().calculate(0, distance))
                .div(distance);
    }

    public double centripetal(double speed, double curvature) {
        return speed * speed * curvature * config.centripetalScaling.get() * config.robotMass.get();
    }

    public double getVelocityToBrakeInTime(double distanceRemaining, double theta) {
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double cos2 = cos * cos;
        double cos3 = cos2 * cos;
        double sin2 = sin * sin;
        double sin3 = sin2 * sin;

        double k1 = config.quadraticBrakeCoefficients.get().get(0, 0) * cos3
                + config.quadraticBrakeCoefficients.get().get(1, 1) * sin3;
        double k2 = config.linearBrakeCoefficients.get().get(0, 0) * cos2
                + config.linearBrakeCoefficients.get().get(1, 1) * sin2;
        Pair<Double, Double> velocityInversion = Utils.solveQuadratic(k1, k2, -distanceRemaining / config.brakeAggression.get());
        return Math.max(velocityInversion.first(), velocityInversion.second());
    }

    public double drive(double tangentialVel, Vector2D closestTangentVector, double heading, double deltaTime, double targetVelocityToBrakeInTime, boolean isBraking, double remainingDistance, double brakingDisplacement) {
        double maxVelocityToFitAccel = tangentialVel + config.maxAccelerationConstraint.get() * deltaTime;
        double constrainedVelocity = Math.min(config.maxVelocityConstraint.get(), maxVelocityToFitAccel);
        double theta = closestTangentVector.angleTo(Vector2D.unit(heading));

        double currentMaxAchievableVelocity = maxAchievableVelocity.radius(theta);

        if (!isBraking)
            if (constrainedVelocity >= currentMaxAchievableVelocity)
                return 1.0;
            else
                return coast(tangentialVel, theta, remainingDistance, constrainedVelocity);

        double targetVel = Math.min(targetVelocityToBrakeInTime, constrainedVelocity);
        double error = targetVel - tangentialVel;

        // TODO: Kalman Filter?
        return config.brakeController.get().calculate(targetVel - excessVelocityAfterBraking(remainingDistance, brakingDisplacement, theta), error);
    }

    public double coast(double tangentialVel, double theta, double remainingDistance, double constrainedVelocity) {
        double targetCoastDecel = maxAchievableDeceleration.radius(theta);
        double coastVelNeededToStopInTime =
                Math.sqrt(config.coastDownToVelocity.get() * config.coastDownToVelocity.get()
                        + 2 * Math.abs(targetCoastDecel) * remainingDistance);

        double zeroPowerCoastFinalVelSquared = tangentialVel * tangentialVel + 2 * targetCoastDecel * remainingDistance;
        double zeroPowerCoastFinalVel =
                Math.signum(zeroPowerCoastFinalVelSquared) * Math.sqrt(Math.abs(zeroPowerCoastFinalVelSquared));
        double targetVel = Math.min(coastVelNeededToStopInTime, constrainedVelocity);

        double velocityMomentumCannotProvide = Math.max(0, (config.coastDownToVelocity.get() - zeroPowerCoastFinalVel));
        double feedforwardVelocity = Math.min(constrainedVelocity, velocityMomentumCannotProvide);

        double error = Math.max(0, targetVel - tangentialVel);
        return config.coastController.get().calculate(feedforwardVelocity, error);
    }

    public Vector2D getBrakeDisplacement(Twist twist, double heading) {
        Vector2D linearTwist = twist.toLinear();

        Vector2D quadratic = linearTwist.hadamardProduct(linearTwist.abs()).transform(config.quadraticBrakeCoefficients.get());
        Vector2D linear = linearTwist.transform(config.linearBrakeCoefficients.get());

        return quadratic.plus(linear).rotate(heading);
    }

    double excessVelocityAfterBraking(
            double availableDisplacement,
            double brakingDisplacement,
            double theta
    ) {
        double overshootDisplacement =
                brakingDisplacement - availableDisplacement;

        boolean stopsBeforeTarget =
                Math.signum(overshootDisplacement)
                        != Math.signum(availableDisplacement);

        if (stopsBeforeTarget) {
            return 0;
        }

        return getVelocityToBrakeInTime(overshootDisplacement, theta);
    }
}
