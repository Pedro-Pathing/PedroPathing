/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.algorithm;

import static com.pedropathing.config.Memoize.memo;
import static com.pedropathing.utils.Angle.normalizeSigned;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Ellipse2D;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathTracker;
import com.pedropathing.utils.Control;
import com.pedropathing.utils.Pair;
import com.pedropathing.utils.Timer;
import com.pedropathing.utils.Utils;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

public class Foresight implements Algorithm {
    public final ForesightConfig config;
    private final Supplier<Ellipse2D> maxAchievableVelocity, maxAchievableDeceleration;
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

        maxAchievableVelocity = memo(
                () -> Ellipse2D.fromAxes(
                        config.maxAchievableForwardVelocity.get(), config.maxAchievableStrafeVelocity.get()),
                config.maxAchievableForwardVelocity,
                config.maxAchievableStrafeVelocity);

        maxAchievableDeceleration = memo(
                () -> Ellipse2D.fromAxes(
                        config.maxAchievableForwardDeceleration.get(), config.maxAchievableStrafeDeceleration.get()),
                config.maxAchievableForwardDeceleration,
                config.maxAchievableStrafeDeceleration);
    }

    @Override
    public DrivePowers calculatePath(Drivetrain drivetrain, PathTracker pathTracker, MotionState state, double deltaTime) {
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
        double thetaForBraking = closestTangent.angleTo(Vector2D.unit(state.pose().heading()));
        Pair<Double, Double> velocityInversion = getVelocityToBrakeInTime(remainingDistance, thetaForBraking);
        double velocityToBrakeInTime = velocityInversion.first();
        double targetAcceleration = velocityInversion.second();
        tangentialSpeed = closestTangent.dot(state.velocity().toVector2D());
        boolean isBraking = tangentialSpeed >= velocityToBrakeInTime;

        double parametricVelocity = tangentialSpeed / pathTracker.current().curve.derivative(closestT).magnitude();
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

        Vector2D displacementToPath = closestPose.minus(state.pose()).toVector2D().projectOnto(closestNormal);
        translationalError = displacementToPath.magnitude();
        Vector2D translationalVector = computeTranslationalCorrection(
                state,
                displacementToPath,
                brakingDisplacement.projectOnto(closestNormal)
        );
        Vector2D normalFeedforward = Vector2D.zero();

        boolean atParametricStart = closestT <= config.parametricTConstraint.get();
        if (atParametricStart) {
            Vector2D displacementToStart =
                    pathTracker.current().curve.startPoint().minus(state.pose().toVector2D());
            double tangentDisplacementToStart = displacementToStart.dot(closestTangent);
            boolean isBeforePath = tangentDisplacementToStart > 1 && translationalError > config.translationalDeviationTolerance.get();
            boolean isHeadingBeforePath = Math.abs(headingError) > config.headingDeviationTolerance.get();

            if (isBeforePath && config.cosineScale.get()) {
                driveVector = driveVector.times(tangentDisplacementToStart / translationalError);
            }

            if (isHeadingBeforePath && config.cosineScale.get()) {
                if (config.turnBeforeDriving.get()) driveVector = Vector2D.zero();
                driveVector = driveVector.times(getDriveScalar(0, headingError));
            }
        } else {
            double centripetal = centripetal(tangentialSpeed, curvature)
                    + config.normalFeedforward.get() * tangentialSpeed;
            normalFeedforward = closestNormal.times(centripetal);

            if (((Math.abs(headingError) > 2 * config.headingDeviationTolerance.get())
                    || (Math.abs(translationalError) > 2 * config.translationalDeviationTolerance.get()))
                    && config.cosineScale.get())
                driveVector = driveVector.times(getDriveScalar(translationalError, headingError));
        }

        return allocatePowers(drivetrain, state, normalFeedforward, headingFeedforward, translationalVector, driveVector, headingPower, translationalError, headingError);
    }

    @Override
    public DrivePowers calculateHold(Drivetrain drivetrain, Pose target, MotionState state, boolean useScaling, double deltaTime) {
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
        }
        else {
            closestTangent = displacementToPath.normalized();
            tangentialSpeed = closestTangent.dot(state.velocity().toVector2D());
        }

        if (busy && testTimeout() || (testHeading() && testTranslational() && testVelocity()))
            busy = false;

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
        return config.headingController.get().calculate(0, headingError, state.twist().omega());
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

        //allocate heading power before + after drive
        headingFeedforward += headingPower * config.headingDriveRatio.get();
        headingPower *= (1 - config.headingDriveRatio.get());

        if (translationalPriority && headingPriority) {
            vectors = Arrays.asList(
                    Pair.of(normalFeedforwardVector, false),
                    Pair.of(Vector2D.polar(headingFeedforward, state.pose().heading()), true),
                    Pair.of(translationalVector, false),
                    Pair.of(Vector2D.polar(headingPower, state.pose().heading()), true),
                    Pair.of(driveVector, false)
            );
        } else if (headingPriority) {
            vectors = Arrays.asList(
                    Pair.of(normalFeedforwardVector, false),
                    Pair.of(Vector2D.polar(headingFeedforward, state.pose().heading()), true),
                    Pair.of(Vector2D.polar(headingPower, state.pose().heading()), true),
                    Pair.of(translationalVector, false),
                    Pair.of(driveVector, false)
            );
        } else {
            vectors = Arrays.asList(
                    Pair.of(normalFeedforwardVector, false),
                    Pair.of(Vector2D.polar(headingFeedforward, state.pose().heading()), true),
                    Pair.of(translationalVector, false),
                    Pair.of(driveVector, false),
                    Pair.of(Vector2D.polar(headingPower, state.pose().heading()), true)
            );
        }

        Pair<Vector2D, Double> clamped = clampPowers(drivetrain, vectors, state);
        return getDrivePowers(clamped.first(), state, clamped.second());
    }

    private Pair<Vector2D, Double> clampPowers(
            Drivetrain drivetrain,
            List<Pair<Vector2D, Boolean>> powers,
            MotionState state) {
        Vector2D pathing = Vector2D.zero();
        double heading = 0.0;

        for (Pair<Vector2D, Boolean> power : powers) {
            boolean isAngular = power.second();

            if (isAngular) {
                Vector2D headingVector = power.first();

                double deltaHeading = headingVector.dot(
                        Vector2D.polar(1.0, state.pose().heading()));

                double scalingFactor = maxScaling(
                        pathing,
                        heading,
                        Vector2D.zero(),
                        deltaHeading,
                        state,
                        drivetrain);

                heading += scalingFactor * deltaHeading;
            } else {
                Vector2D vector = power.first();

                double scalingFactor = maxScaling(
                        pathing,
                        heading,
                        vector,
                        0.0,
                        state,
                        drivetrain);

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

    public Vector2D computeTranslationalCorrection(MotionState state, Vector2D displacementVector, Vector2D brakingDisplacement) {
        if (displacementVector == null || displacementVector.isZero()) return Vector2D.zero();
        Vector2D adjustedError = displacementVector.minus(brakingDisplacement);
        double distance = adjustedError.magnitude();
        if (distance < config.minCorrectionDistance.get()) return Vector2D.zero();
        return adjustedError
                .times(getTranslationalCorrection(state, adjustedError))
                .div(distance);
    }

    private double getTranslationalCorrection(MotionState state, Vector2D error) {
        Vector2D bodyFrameError = error.toBodyFrame(state.pose().heading());
        double distance = bodyFrameError.magnitude();
        double forwardCorrection = config.forwardTranslationalController.get().calculate(0, distance);
        double lateralCorrection = config.lateralTranslationalController.get().calculate(0, distance);
        return Ellipse2D.interpolateRadius(forwardCorrection, lateralCorrection, error.theta());
    }

    public double centripetal(double speed, double curvature) {
        return speed * speed * curvature * config.centripetalScaling.get() * config.robotMass.get();
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
        if (config.fullPowerCoast.get()) {
            double error = targetVelocityToBrakeInTime - tangentialVel;
            if (!isBraking || (error > 0)) {
                targetVelocity = 0;
                return 1.0;
            }
            double theta = closestTangentVector.angleTo(Vector2D.unit(heading));
            targetVelocity = targetVelocityToBrakeInTime;
            return config.brakeController
                    .get()
                    .calculate(targetVelocityToBrakeInTime - excessVelocityAfterBraking(remainingDistance, brakingDisplacement, theta), error) +
                    config.brakeAccelFeedforward.get().calculate(targetAccel, 0);
        }

        double maxVelocityToFitAccel = tangentialVel + config.maxAccelerationConstraint.get() * deltaTime;
        double constrainedVelocity = Math.min(config.maxVelocityConstraint.get(), maxVelocityToFitAccel);
        double theta = closestTangentVector.angleTo(Vector2D.unit(heading));

        double currentMaxAchievableVelocity = maxAchievableVelocity.get().radius(theta);

        if (!isBraking)
            if (constrainedVelocity >= currentMaxAchievableVelocity) {
                targetVelocity = 0;
                return 1.0;
            }
            else return coast(tangentialVel, theta, remainingDistance, constrainedVelocity);

        targetVelocity = Math.min(targetVelocityToBrakeInTime, constrainedVelocity);
        double error = targetVelocity - tangentialVel;

        return config.brakeController
                .get()
                .calculate(targetVelocity - excessVelocityAfterBraking(remainingDistance, brakingDisplacement, theta), error) +
               config.brakeAccelFeedforward.get().calculate(targetAccel, 0);
    }

    public double maxScaling(Vector2D translation,
                             double heading,
                             Vector2D deltaTranslation,
                             double deltaHeading,
                             MotionState state,
                             Drivetrain drivetrain) {
        DrivePowers current = getDrivePowers(
                translation,
                state,
                heading);

        DrivePowers delta = getDrivePowers(
                deltaTranslation,
                state,
                deltaHeading);

        return drivetrain.maxScaling(current, delta);
    }


    public double coast(double tangentialVel, double theta, double remainingDistance, double constrainedVelocity) {
        double targetCoastDecel = -Math.abs(maxAchievableDeceleration.get().radius(theta));
        double coastVelNeededToStopInTime = Math.sqrt(config.coastDownToVelocity.get() * config.coastDownToVelocity.get()
                        - 2 * targetCoastDecel * remainingDistance);

        double zeroPowerCoastFinalVelSquared = excessVelAfterCoast(remainingDistance, tangentialVel, theta);
        double zeroPowerCoastFinalVel =
                Math.signum(zeroPowerCoastFinalVelSquared) * Math.sqrt(Math.abs(zeroPowerCoastFinalVelSquared));
        double targetVel = Math.min(coastVelNeededToStopInTime, constrainedVelocity);

        double velocityMomentumCannotProvide = Math.max(0, (config.coastDownToVelocity.get() - zeroPowerCoastFinalVel));
        double feedforwardVelocity = Math.min(constrainedVelocity, velocityMomentumCannotProvide);

        double error = Math.max(0, targetVel - tangentialVel);

        return config.coastController.get().calculate(feedforwardVelocity, error);
    }

    public Vector2D getBrakeDisplacement(Twist twist, double heading) {
        Vector2D linearTwist = twist.toVector2D();

        Vector2D quadratic =
                linearTwist.hadamardProduct(linearTwist.abs()).transform(config.quadraticBrakeCoefficients.get());
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
        double accel = Math.abs(maxAchievableDeceleration.get().radius(theta));
        return Math.sqrt(initialVelocity * initialVelocity - 2 * accel * remainingDistance);
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
