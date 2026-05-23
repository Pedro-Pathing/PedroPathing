package com.pedropathing.algorithm;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.FollowState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.PathProgress;
import com.pedropathing.utils.Utils.Control;
import com.pedropathing.utils.Utils.Angle;
import com.pedropathing.utils.Utils;
import com.pedropathing.utils.Pair;

public class Foresight implements Algorithm {
    private final ForesightConfig config;

    public Foresight(ForesightConfig config) {
        this.config = config;
    }

    @Override
    public DrivePowers calculate(FollowState state) {
        double headingError = headingError(state.motionState().motionState().pose().heading(),
                state.getTargetHeading());
        double headingPower = config.headingController.get().calculate(state.getTargetHeading(), headingError);

        if (state.isAtParametricEnd()) {
            return holdPoint(state.getCurve().endPoint(), state, headingPower);
        }
        if (state.isBeforeParametricStart()) {
            return holdPoint(state.getCurve().startPoint(), state, headingPower);
        }

        double velocityToBrakeInTime =
                getVelocityToBrakeInTime(state.getPathProgress().distanceRemaining,
                        state.motionState().pose().heading());
        boolean isBraking = state.getTangentialSpeed() >= velocityToBrakeInTime;
        // may want hard switch? or maybe add some hysteresis?
        // or hard switch until velocity is going to change directions if it continues to brake?

        boolean pathSkip =
                isBraking && (!state.isLastPath() || !config.shouldBrakeAtEnd.get());

        if (pathSkip) {
            state.advanceToNextPath();
            return calculate(state);
        }

        double tangentPower = tangent(state.getTangentialSpeed(), state.getPathProgress(),
                state.motionState().pose().heading(), state.getDeltaTime(), velocityToBrakeInTime, isBraking);

        double normalError = normalError(state.motionState().pose(), state.getPathProgress());
        double normalPower =
                computeTranslationalCorrection(
                        state.getPathProgress().normal.times(normalError),
                        state.motionState().velocity(),
                        state.motionState().pose().heading()).dot(state.getPathProgress().normal);
        double centripetal = centripetal(state.getTangentialSpeed(),
                state.getPathProgress());
        normalPower = normalPower + centripetal;

        tangentPower *= getTangentScalar(normalError, headingError);

        return allocatePowers(state, normalPower, tangentPower, headingPower);
    }

    public DrivePowers holdPoint(Vector2D target, FollowState state, double headingPower) {
        Vector2D translationalError =
                target.minus(state.motionState().pose().toVector2D());
        Vector2D translational = computeTranslationalCorrection(translationalError,
                state.motionState().velocity(), state.motionState().pose().heading());
        return getDrivePowers(translational, state, headingPower);
    }

    /**
     * Gives a tangent scalar to scale down the tangent power based on the normal and
     * heading errors. This is to prevent aggressive tangent correction when the robot
     * is deviating a lot from the path or facing the wrong direction.
     */
    public double getTangentScalar(double normalError, double headingError) {
        double trackDeviationScale = Control.cosineScale(normalError, config.headingDeviationTolerance.get());
        double headingScale = Control.cosineScale(headingError, config.lateralDeviationTolerance.get());
        return trackDeviationScale * headingScale;
    }

    public DrivePowers allocatePowers(FollowState state, double normalPower, double tangentPower, double headingPower) {
        double magnitudeRemaining = 1;
        double normalUsed = Control.allocatePower(normalPower, magnitudeRemaining);
        double remaining = Control.getRemainingMagnitude(magnitudeRemaining, normalUsed);
        double headingUsed = Control.allocatePower(headingPower, remaining);
        remaining = Control.getRemainingMagnitude(remaining, headingUsed);
        double tangentUsed =
                Control.allocatePower(tangentPower, Math.min(1, remaining));

        Vector2D drivePower =
                state.getPathProgress().normal.times(normalUsed)
                        .plus(state.getPathProgress().tangent.times(tangentUsed));

        return getDrivePowers(drivePower, state, headingUsed);
    }

    public DrivePowers getDrivePowers(Vector2D fieldRelativeDrivePower, FollowState state, double headingPower) {
        Vector2D robotFrameDrivePower =
                fieldRelativeDrivePower.rotate(-state.motionState().pose().heading());
        double forward = Control.clampBrakingPower(robotFrameDrivePower.x(), state.motionState().twist().vx(), config.maxBrakingPower.get());
        double strafe = Control.clampBrakingPower(robotFrameDrivePower.y(), state.motionState().twist().vy(), config.maxBrakingPower.get());

        return new DrivePowers(forward, strafe, headingPower);
    }

    public double headingError(double current, double target) {
        return Angle.smallestDifference(current, target) * Angle.turnDirection(current, target);
    }

    public double normalError(Pose currentPose, PathProgress pathProgress) {
        return currentPose.toVector2D().minus(pathProgress.point).dot(pathProgress.normal);
    }

    private Vector2D computeTranslationalCorrection(Vector2D displacementVector, Velocity velocity, double currentHeading) {
        Vector2D linearVel = velocity.toLinear().projectOnto(displacementVector);
        double theta = linearVel.angleTo(Vector2D.unit(currentHeading));
        Vector2D brakingDisplacement = getBrakeDisplacement(linearVel.dot(displacementVector), theta);
        Vector2D adjustedError = displacementVector.minus(brakingDisplacement);
        double distance = adjustedError.magnitude();
        if (distance < config.minCorrectionDistance.get()) return Vector2D.zero();
        return adjustedError.times(config.translationalController.get().calculate(0, distance)).div(distance);
    }

    public double centripetal(double speed, PathProgress pathProgress) {
        return speed * speed * pathProgress.curvature * config.centripetalScaling.get();
    }

    public double getVelocityToBrakeInTime(double distanceRemaining, double theta) {
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double cos2 = cos * cos;
        double cos3 = cos2 * cos;
        double sin2 = sin * sin;
        double sin3 = sin2 * sin;

        double k1 = config.quadraticBrakeCoefficients.get().get(0, 0) * cos3 + config.quadraticBrakeCoefficients.get().get(1, 1) * sin3;
        double k2 = config.linearBrakeCoefficients.get().get(0, 0) * cos2 + config.linearBrakeCoefficients.get().get(1, 1) * sin2;
        double velocityToBrakeToInDistance = getBrakeDisplacement(config.velocityToBrakeTo.get(), theta).dot(Vector2D.unit(theta));
        Pair<Double, Double> velocityInversion = Utils.solveQuadratic(k1, k2, -distanceRemaining /
                config.brakingOvershootBias.get() + velocityToBrakeToInDistance);
        return Math.max(velocityInversion.first(), velocityInversion.second());
    }

    public double tangent(double tangentialVel, PathProgress pathProgress, double heading, double deltaTime, double targetVelocityToBrakeInTime, boolean isBraking) {
        double maxVelocityToFitAccel =
                tangentialVel + config.maxAcceleration.get() * deltaTime;
        double constrainedVelocity = Math.min(config.maxVelocity.get(), maxVelocityToFitAccel);
        double theta = pathProgress.tangent.angleTo(Vector2D.unit(heading));

        double currentMaxAchievableVelocity = config.maxAchievableVelocity.get().radius(theta);
        constrainedVelocity = Math.min(constrainedVelocity, currentMaxAchievableVelocity);

        if (!isBraking) {
            return coast(tangentialVel, theta, pathProgress, constrainedVelocity);
        }

        double targetVel = Math.min(targetVelocityToBrakeInTime, constrainedVelocity);
        double error = targetVel - tangentialVel;

        //TODO: Kalman Filter?
        return config.brakeController.get().calculate(targetVel, error);
    }

    public double coast(double tangentialVel, double theta, PathProgress pathProgress, double constrainedVelocity) {
        double targetCoastDecel = config.coastingDecelerationConstraint.get().radius(theta);
        double coastVelNeededToStopInTime =
                Math.sqrt(config.coastDownToVelocity.get() * config.coastDownToVelocity.get() +
                        2 * Math.abs(targetCoastDecel) * pathProgress.distanceRemaining);

        double zeroPowerCoastFinalVelSquared =
                tangentialVel * tangentialVel + 2 * config.naturalDeceleration.get().radius(theta) * pathProgress.distanceRemaining;
        double zeroPowerCoastFinalVel =
                Math.signum(zeroPowerCoastFinalVelSquared) * Math.sqrt(Math.abs(zeroPowerCoastFinalVelSquared));
        double targetVel = Math.min(coastVelNeededToStopInTime, constrainedVelocity);

        double velocityMomentumCannotProvide = Math.max(0,
                config.coastDownToVelocity.get() - zeroPowerCoastFinalVel);
        double feedforwardVelocity = Math.min(constrainedVelocity, velocityMomentumCannotProvide);

        double error = Math.max(0, targetVel - tangentialVel);
        return config.coastController.get().calculate(feedforwardVelocity, error);
    }

    private Vector2D getBrakeDisplacement(double v, double theta) {
        Vector2D unit = Vector2D.unit(theta);
        Vector2D quadraticTerm =
                unit.hadamardProduct(unit).transform(config.quadraticBrakeCoefficients.get()).times(v * Math.abs(v));
        Vector2D linearTerm = unit.transform(config.linearBrakeCoefficients.get()).times(v);
        return quadraticTerm.plus(linearTerm);
    }
}