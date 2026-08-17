/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.algorithm;

import static com.pedropathing.utils.Angle.normalizeSigned;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.*;
import com.pedropathing.paths.PathTracker;
import com.pedropathing.utils.Control;
import com.pedropathing.utils.Pair;
import com.pedropathing.utils.Timer;
import com.pedropathing.utils.Utils;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Foresight implements Algorithm {
    public final ForesightConfig config;
    private double closestT, curvature;
    private double curveCompletion, remainingDistance, tangentialSpeed;
    private Pose closestPose;
    private Vector2D closestTangent, closestNormal;
    private final Timer timer = new Timer();
    private boolean resetTimer = true;
    private double headingError, translationalError, targetVelocity;
    private boolean busy = false;

    public Foresight(ForesightConfig config) {
        this.config = config;
    }

    @Override
    public DrivePowers calculatePath(
            Drivetrain drivetrain, PathTracker pathTracker, MotionState state, double deltaTime) {
        double targetHeading;

        if (testParametric()) { // End Constraint
            closestT = 1.0;
            targetHeading = pathTracker.current().heading(closestT);
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
            closestT = pathTracker.current().curve.closestT(state.pose().toVector2D(), closestT);
            targetHeading = pathTracker.current().heading(closestT);
            closestPose = pathTracker.current().curve.get(closestT).toPose(targetHeading);
            curvature = pathTracker.current().curve.curvature(closestT);
        }

        headingError = headingError(state.pose().heading(), targetHeading);
        double headingPower = headingPower(headingError, state);
        remainingDistance = pathTracker.current().curve.remainingDistance(closestT);
        curveCompletion = 1 - remainingDistance / pathTracker.current().curve.length();

        closestTangent = pathTracker.current().curve.tangent(closestT);
        closestNormal = pathTracker.current().curve.leftNormal(closestT);
        double thetaForBraking =
                closestTangent.angleTo(Vector2D.unit(state.pose().heading()));
        Pair<Double, Double> velocityInversion = getVelocityToBrakeInTime(remainingDistance, thetaForBraking);
        double velocityToBrakeInTime = velocityInversion.first();
        double targetAcceleration = velocityInversion.second();
        tangentialSpeed = closestTangent.dot(state.velocity().toVector2D());
        boolean isBraking = tangentialSpeed >= velocityToBrakeInTime;

        double parametricVelocity = tangentialSpeed
                / pathTracker.current().curve.derivative(closestT).magnitude();
        double interpolationDerivative = pathTracker.current().headingDerivative(closestT);
        double headingDerivative = interpolationDerivative * parametricVelocity;
        double headingFeedforward = config.headingFeedforward.get().calculate(headingDerivative, 0);

        boolean pathSkip = isBraking && (pathTracker.remainingPaths() > 1 || !config.brakeAtEnd.get());

        if (pathSkip) {
            pathTracker.advance();
            return calculatePath(drivetrain, pathTracker, state, deltaTime);
        }

        Vector2D brakingDisplacement =
                getBrakeDisplacement(state.twist(), state.pose().heading());

        Vector2D driveVector = closestTangent.times(drive(
                tangentialSpeed,
                closestTangent,
                state.pose().heading(),
                deltaTime,
                velocityToBrakeInTime,
                isBraking,
                remainingDistance,
                brakingDisplacement.dot(closestTangent),
                targetAcceleration));

        Vector2D displacementToPath =
                closestPose.minus(state.pose()).toVector2D().projectOnto(closestNormal);
        translationalError = displacementToPath.magnitude();
        Vector2D translationalVector = computeTranslationalCorrection(
                state, displacementToPath, brakingDisplacement.projectOnto(closestNormal));
        Vector2D normalFeedforward = Vector2D.zero();

        boolean atParametricStart = closestT <= config.parametricTConstraint.get();
        if (atParametricStart) {
            Vector2D displacementToStart =
                    pathTracker.current().curve.startPoint().minus(state.pose().toVector2D());
            double tangentDisplacementToStart = displacementToStart.dot(closestTangent);
            boolean isBeforePath =
                    tangentDisplacementToStart > 1 && translationalError > config.translationalDeviationTolerance.get();
            boolean isHeadingBeforePath = Math.abs(headingError) > config.headingDeviationTolerance.get();

            if (isBeforePath && config.cosineScale.get()) {
                driveVector = driveVector.times(tangentDisplacementToStart / translationalError);
            }

            if (isHeadingBeforePath && config.cosineScale.get()) {
                if (config.turnBeforeDriving.get()) driveVector = Vector2D.zero();
                driveVector = driveVector.times(getDriveScalar(0, headingError));
            }
        } else {
            normalFeedforward = centripetalEffort(tangentialSpeed, curvature, state.pose().heading()).projectOnto(closestNormal);

            if (((Math.abs(headingError) > 2 * config.headingDeviationTolerance.get())
                            || (Math.abs(translationalError) > 2 * config.translationalDeviationTolerance.get()))
                    && config.cosineScale.get())
                driveVector = driveVector.times(getDriveScalar(translationalError, headingError));
        }

        return allocatePowers(
                drivetrain,
                state,
                normalFeedforward,
                headingFeedforward,
                translationalVector,
                driveVector,
                headingPower,
                translationalError,
                headingError);
    }

    @Override
    public DrivePowers calculateHold(
            Drivetrain drivetrain, Pose target, MotionState state, boolean useScaling, double deltaTime) {
        if (resetTimer) {
            timer.reset();
            resetTimer = false;
        }

        closestPose = target;
        headingError = headingError(state.pose().heading(), target.heading());
        Vector2D displacementToPath = closestPose.minus(state.pose()).toVector2D();
        translationalError = displacementToPath.magnitude();

        if (displacementToPath.isZero()) {
            tangentialSpeed = 0;
            closestTangent = Vector2D.zero();
        } else {
            closestTangent = displacementToPath.normalized();
            tangentialSpeed = closestTangent.dot(state.velocity().toVector2D());
        }

        if (busy && testTimeout() || (testHeading() && testTranslational() && testVelocity())) busy = false;

        Vector2D translational = computeTranslationalCorrection(
                state,
                displacementToPath,
                getBrakeDisplacement(state.twist(), state.pose().heading()));
        double headingPower = headingPower(headingError, state);

        if (useScaling) {
            double translationalScale = config.holdPointTranslationalScaling.get();
            double headingScale = config.holdPointHeadingScaling.get();

            translational = translational.times(translationalScale);
            headingPower *= headingScale;
        }

        return getDrivePowers(translational, state, headingPower);
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

    /**
     * Compute heading correction power for the given state and target heading.
     */
    public double headingPower(double headingError, MotionState state) {
        return config.headingController
                .get()
                .calculate(0, headingError, state.twist().omega());
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

    @SuppressWarnings("unchecked")
    public DrivePowers allocatePowers(
            Drivetrain drivetrain,
            MotionState state,
            Vector2D normalFeedforwardVector,
            double headingFeedforward,
            Vector2D translationalVector,
            Vector2D driveVector,
            double headingPower,
            double translationalError,
            double headingError) {
        boolean translationalPriority = Math.abs(translationalError) > config.translationalDeviationTolerance.get();
        boolean headingPriority = Math.abs(headingError) > config.headingDeviationTolerance.get();

        List<Pair<Vector2D, Boolean>> vectors;

        // allocate heading power before + after drive
        headingFeedforward += headingPower * config.headingDriveRatio.get();
        headingPower *= (1 - config.headingDriveRatio.get());

        if (translationalPriority && headingPriority) {
            vectors = Arrays.asList(
                    Pair.of(normalFeedforwardVector, false),
                    Pair.of(Vector2D.polar(headingFeedforward, state.pose().heading()), true),
                    Pair.of(translationalVector, false),
                    Pair.of(Vector2D.polar(headingPower, state.pose().heading()), true),
                    Pair.of(driveVector, false));
        } else if (headingPriority) {
            vectors = Arrays.asList(
                    Pair.of(normalFeedforwardVector, false),
                    Pair.of(Vector2D.polar(headingFeedforward, state.pose().heading()), true),
                    Pair.of(Vector2D.polar(headingPower, state.pose().heading()), true),
                    Pair.of(translationalVector, false),
                    Pair.of(driveVector, false));
        } else {
            vectors = Arrays.asList(
                    Pair.of(normalFeedforwardVector, false),
                    Pair.of(Vector2D.polar(headingFeedforward, state.pose().heading()), true),
                    Pair.of(translationalVector, false),
                    Pair.of(driveVector, false),
                    Pair.of(Vector2D.polar(headingPower, state.pose().heading()), true));
        }

        Pair<Vector2D, Double> clamped = clampPowers(drivetrain, vectors, state);
        return getDrivePowers(clamped.first(), state, clamped.second());
    }

    private Pair<Vector2D, Double> clampPowers(
            Drivetrain drivetrain, List<Pair<Vector2D, Boolean>> powers, MotionState state) {
        Vector2D pathing = Vector2D.zero();
        double heading = 0.0;

        for (Pair<Vector2D, Boolean> power : powers) {
            boolean isAngular = power.second();

            if (isAngular) {
                Vector2D headingVector = power.first();

                double deltaHeading =
                        headingVector.dot(Vector2D.polar(1.0, state.pose().heading()));

                double scalingFactor = maxScaling(pathing, heading, Vector2D.zero(), deltaHeading, state, drivetrain);

                heading += scalingFactor * deltaHeading;
            } else {
                Vector2D vector = power.first();

                double scalingFactor = maxScaling(pathing, heading, vector, 0.0, state, drivetrain);

                Vector2D scaled = vector.times(scalingFactor);
                pathing = pathing.plus(scaled);
            }
        }

        return Pair.of(pathing, heading);
    }

    public DrivePowers getDrivePowers(Vector2D fieldRelativeDrivePower, MotionState state, double headingPower) {
        Vector2D robotFrameDrivePower =
                fieldRelativeDrivePower.rotate(-state.pose().heading());
        double forward = Control.clampBrakingPower(
                robotFrameDrivePower.x(), state.twist().vx(), config.maxBrakingPower.get());
        double strafe = Control.clampBrakingPower(
                robotFrameDrivePower.y(), state.twist().vy(), config.maxBrakingPower.get());
        return new DrivePowers(forward, strafe, headingPower);
    }

    public double headingError(double current, double target) {
        return normalizeSigned(target - current);
    }

    public Vector2D computeTranslationalCorrection(
            MotionState state, Vector2D displacementVector, Vector2D brakingDisplacement) {
        if (displacementVector == null || displacementVector.isZero()) return Vector2D.zero();
        Vector2D adjustedError = displacementVector.minus(brakingDisplacement);
        double distance = adjustedError.magnitude();
        if (distance < config.minCorrectionDistance.get()) return Vector2D.zero();

        Vector2D bodyFrameError = adjustedError.toBodyFrame(state.pose().heading());
        return Vector2D.cartesian(
                config.forwardTranslationalController.get().calculate(0, bodyFrameError.x()),
                config.lateralTranslationalController.get().calculate(0, bodyFrameError.y())
        ).projectOnto(bodyFrameError).toWorldFrame(state.pose().heading());
    }

    public Vector2D centripetalEffort(double speed, double curvature, double heading) {
        double desiredCentripetalAccel = speed * speed * curvature;
        Matrix headingMatrix = Matrix.rotation(heading);
        Vector2D worldFrameCentripetalGains = config.centripetalGains.get().transform(headingMatrix);
        return worldFrameCentripetalGains.times(desiredCentripetalAccel);
    }

    public Pair<Double, Double> getVelocityToBrakeInTime(double distanceRemaining, double theta) {
        double cos = Math.abs(Math.cos(theta));
        double sin = Math.abs(Math.sin(theta));
        double cos2 = cos * cos;
        double cos3 = cos2 * cos;
        double sin2 = sin * sin;
        double sin3 = sin2 * sin;

        double k1 = config.quadraticBrakeCoefficients.get().get(0, 0) * cos3
                + config.quadraticBrakeCoefficients.get().get(1, 1) * sin3;
        double k2 = config.linearBrakeCoefficients.get().get(0, 0) * cos2
                + config.linearBrakeCoefficients.get().get(1, 1) * sin2;
        Pair<Double, Double> velocityInversion =
                Utils.solveQuadratic(k1, k2, -distanceRemaining / config.brakeAggression.get());
        double maxVel = Math.max(velocityInversion.first(), velocityInversion.second());
        return Pair.of(maxVel, -maxVel / (2 * maxVel * k1 + k2));
    }

    public double drive(
            double tangentialVel,
            Vector2D closestTangentVector,
            double heading,
            double deltaTime,
            double targetVelocityToBrakeInTime,
            boolean isBraking,
            double remainingDistance,
            double brakingDisplacement,
            double targetAccel) {
        double headingFromTangent = closestTangentVector.angleTo(Vector2D.unit(heading));
        double maxAchievableVelocity = DiamondDrivetrainModel.interpolateVelocity(config.maxAchievableForwardVelocity.get(), config.maxAchievableStrafeVelocity.get(), headingFromTangent);

        if (!isBraking) return coast(tangentialVel, headingFromTangent, remainingDistance, deltaTime, maxAchievableVelocity);

        targetVelocity = Math.min(targetVelocityToBrakeInTime, maxAchievableVelocity);
        double error = targetVelocity - tangentialVel;

        return config.brakeController
                .get()
                .calculate(targetVelocity - excessVelocityAfterBraking(remainingDistance, brakingDisplacement, headingFromTangent), error) +
               config.brakeAccelFeedforward.get().calculate(targetAccel, 0);
    }

    public double maxScaling(
            Vector2D translation,
            double heading,
            Vector2D deltaTranslation,
            double deltaHeading,
            MotionState state,
            Drivetrain drivetrain) {
        DrivePowers current = getDrivePowers(translation, state, heading);
        DrivePowers delta = getDrivePowers(deltaTranslation, state, deltaHeading);

        return drivetrain.maxScaling(current, delta);
    }

    public double coast(double tangentialVel, double headingFromTangent, double remainingDistance, double deltaTime, double maxAchievableVelocity) {
        double maxAccelerationConstraint = config.maxAccelerationConstraint.get();
        double maxVelocityConstraint = config.maxVelocityConstraint.get();
        double maxDecelerationConstraint = config.maxDecelerationConstraint.get();
        double coastDownToVelocity = config.coastDownToVelocity.get();

        double targetVel = maxAchievableVelocity;
        if (maxVelocityConstraint != ForesightConfig.Constraint.NONE) {
            targetVel = Math.min(targetVel, maxVelocityConstraint);
        }
        if (maxAccelerationConstraint != ForesightConfig.Constraint.NONE) {
            targetVel = Math.min(targetVel, tangentialVel + maxAccelerationConstraint * deltaTime);
        }

        double feedforwardVelocity = targetVel;

        if (maxDecelerationConstraint != ForesightConfig.Constraint.NONE) {
            double velocityNeededToCoastInTime = Math.sqrt(coastDownToVelocity * coastDownToVelocity - 2 * -maxDecelerationConstraint * remainingDistance);
            double velocityMomentumCannotProvide = Math.max(0, (coastDownToVelocity - excessVelAfterCoast(remainingDistance, tangentialVel, headingFromTangent)));
            targetVel = Math.min(targetVel, velocityNeededToCoastInTime);
            feedforwardVelocity = Math.min(feedforwardVelocity, velocityMomentumCannotProvide);
        }

        if (targetVel >= maxAchievableVelocity) {
            targetVelocity = maxAchievableVelocity;
            return 1;
        }

        double error = Math.max(0, targetVel - tangentialVel);
        targetVelocity = targetVel;
        return config.coastController.get().calculate(feedforwardVelocity, error);
    }

    public Vector2D getBrakeDisplacement(Twist twist, double heading) {
        Vector2D linearTwist = twist.toVector2D();
        Vector2D quadratic = linearTwist.hadamardProduct(linearTwist.abs()).transform(config.quadraticBrakeCoefficients.get());
        Vector2D linear = linearTwist.transform(config.linearBrakeCoefficients.get());
        return quadratic.plus(linear).rotate(heading);
    }

    private double excessVelocityAfterBraking(double availableDisplacement, double brakingDisplacement, double theta) {
        double overshootDisplacement = brakingDisplacement - availableDisplacement;

        boolean stopsBeforeTarget = Math.signum(overshootDisplacement) != Math.signum(availableDisplacement);

        if (stopsBeforeTarget) {
            return 0;
        }

        return getVelocityToBrakeInTime(overshootDisplacement, theta).first();
    }

    private double excessVelAfterCoast(double remainingDistance, double initialVelocity, double theta) {
        double naturalDeceleration = DiamondDrivetrainModel.interpolateAcceleration(config.naturalForwardDeceleration.get(), config.naturalStrafeDeceleration.get(), theta);
        double excessVelocitySquared = initialVelocity * initialVelocity - 2 * naturalDeceleration * remainingDistance;
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
    public boolean isBusy() {
        return busy;
    }
}
