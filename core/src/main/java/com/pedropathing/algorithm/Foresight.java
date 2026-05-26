package com.pedropathing.algorithm;

import com.pedropathing.config.Memoize;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.FollowState;
import com.pedropathing.math.Ellipse2D;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Utils.Control;
import com.pedropathing.utils.Utils.Angle;
import com.pedropathing.utils.Utils;
import com.pedropathing.utils.Pair;

import java.util.Arrays;
import java.util.List;

public class Foresight implements Algorithm {
    private final ForesightConfig config;
    private final Ellipse2D maxAchievableVelocity, maxAchievableDeceleration;
    public final Memoize<Pair<Ellipse2D, Double>, Ellipse2D> coastingDecelerationConstraint;

    public Foresight(ForesightConfig config) {
        this.config = config;

        maxAchievableVelocity = Ellipse2D.fromAxes(config.maxAchievableForwardVelocity.get(), config.maxAchievableStrafeVelocity.get());
        maxAchievableDeceleration = Ellipse2D.fromAxes(config.maxAchievableForwardDeceleration.get(), config.maxAchievableStrafeDeceleration.get());
        coastingDecelerationConstraint = Memoize.memo(
                () -> Pair.of(maxAchievableDeceleration.get(), coastingConstraintScale.get()),
                p -> Ellipse2D.fromAxes(
                        -Math.abs(p.first().getMajorAxis()) * p.second(),
                        -Math.abs(p.first().getMinorAxis()) * p.second()
                )
        );
    }

    @Override
    public DrivePowers calculate(FollowState state) {
        double headingError = headingError(state.motionState().pose().heading(), state.getTargetHeading());
        double headingPower = config.headingController.get().calculate(state.getTargetHeading(), headingError);

        if (state.isAtParametricEnd())
            return holdPoint(state.getCurve().endPoint(), state, headingPower);

        if (state.isBeforeParametricStart())
            return holdPoint(state.getCurve().startPoint(), state, headingPower);


        double velocityToBrakeInTime = getVelocityToBrakeInTime(state.getPathProgress().distanceRemaining, state.motionState().pose().heading());
        boolean isBraking = state.getTangentialSpeed() >= velocityToBrakeInTime;
        // may want hard switch? or maybe add some hysteresis?
        // or hard switch until velocity is going to change directions if it continues to brake?

        boolean pathSkip = isBraking && (!state.isLastPath() || !config.brakeAtEnd.get());

        if (pathSkip) {
            state.advanceToNextPath();
            return calculate(state);
        }

        double drivePower = drive(state.getTangentialSpeed(), state.getPathProgress(), state.motionState().pose().heading(), state.getDeltaTime(), velocityToBrakeInTime, isBraking);

        double translationalError = translationalError(state.motionState().pose(), state.getPathProgress());
        double translationalPower = computeTranslationalCorrection(state.getPathProgress().normal.times(translationalError), state.motionState().velocity(), state.motionState().pose().heading()).dot(state.getPathProgress().normal);
        double centripetal = centripetal(state.getTangentialSpeed(), state.getPathProgress());
        translationalPower = translationalPower + centripetal;

        if ((headingError > 2*config.headingDeviationTolerance.get()) || (translationalError > 2*config.translationalDeviationTolerance.get()))
            drivePower *= getDriveScalar(translationalError, headingError);

        return allocatePowers(state, translationalPower, drivePower, headingPower);
    }

    public DrivePowers holdPoint(Vector2D target, FollowState state, double headingPower) {
        Vector2D translationalError = target.minus(state.motionState().pose().toVector2D());
        Vector2D translational = computeTranslationalCorrection(translationalError, state.motionState().velocity(), state.motionState().pose().heading());
        return getDrivePowers(translational, state, headingPower);
    }

    /**
     * Gives a drive scalar to scale down the drive power based on the translational and
     * heading errors. This is to prevent aggressive drive correction when the robot
     * is deviating a lot from the path or facing the wrong direction.
     */
    public double getDriveScalar(double normalError, double headingError) {
        double trackDeviationScale = Control.cosineScale(normalError, config.headingDeviationTolerance.get());
        double headingScale = Control.cosineScale(headingError, config.translationalDeviationTolerance.get());
        return trackDeviationScale * headingScale;
    }

    private static final int TRANSLATIONAL = 0;
    private static final int HEADING = 1;
    private static final int DRIVE = 2;

    public DrivePowers allocatePowers(FollowState state, double translationalPower, double drivePower, double headingPower,
                                      double translationalError, double headingError) {
        boolean translationalPriority = translationalError > config.translationalDeviationTolerance.get();
        boolean headingPriority = headingError > config.headingDeviationTolerance.get();

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

        Vector2D fieldRelativeDrivePower = state.getPathProgress().normal
                .times(powers[prioritization[TRANSLATIONAL]])
                .plus(state.getPathProgress().tangent.times(powers[prioritization[DRIVE]]));

        return getDrivePowers(fieldRelativeDrivePower, state, powers[prioritization[HEADING]]);
    }

    private double[] clampPowers(double[] powers) {
        double magnitudeRemaining = 1.0;
        double[] usedPowers = new double[3];

        for (int i = 0; i < usedPowers.length; i++) {
            double used = Control.allocatePower(powers[0], magnitudeRemaining);
            magnitudeRemaining = Control.getRemainingMagnitude(magnitudeRemaining, used);
            usedPowers[i] = used;
        }

        return usedPowers;
    }

    public DrivePowers getDrivePowers(Vector2D fieldRelativeDrivePower, FollowState state, double headingPower) {
        Vector2D robotFrameDrivePower = fieldRelativeDrivePower.rotate(-state.motionState().pose().heading());
        double forward = Control.clampBrakingPower(robotFrameDrivePower.x(), state.motionState().twist().vx(), config.maxBrakingPower.get());
        double strafe = Control.clampBrakingPower(robotFrameDrivePower.y(), state.motionState().twist().vy(), config.maxBrakingPower.get());
        return new DrivePowers(forward, strafe, headingPower);
    }

    public double headingError(double current, double target) {
        return Angle.smallestDifference(current, target) * Angle.turnDirection(current, target);
    }

    public double translationalError(Pose currentPose, PathProgress pathProgress) {
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
        Pair<Double, Double> velocityInversion = Utils.solveQuadratic(k1, k2, -distanceRemaining / (config.brakingOvershootBias.get() + velocityToBrakeToInDistance));
        return Math.max(velocityInversion.first(), velocityInversion.second());
    }

    public double drive(double tangentialVel, PathProgress pathProgress, double heading, double deltaTime, double targetVelocityToBrakeInTime, boolean isBraking) {
        double maxVelocityToFitAccel = tangentialVel + config.maxAccelerationConstraint.get() * deltaTime;
        double constrainedVelocity = Math.min(config.maxVelocityConstraint.get(), maxVelocityToFitAccel);
        double theta = pathProgress.tangent.angleTo(Vector2D.unit(heading));

        double currentMaxAchievableVelocity = maxAchievableVelocity.radius(theta);

        if (!isBraking)
            if (constrainedVelocity >= currentMaxAchievableVelocity)
                return 1.0;
            else
                return coast(tangentialVel, theta, pathProgress, constrainedVelocity);

        double targetVel = Math.min(targetVelocityToBrakeInTime, constrainedVelocity);
        double error = targetVel - tangentialVel;

        //TODO: Kalman Filter?
        return config.brakeController.get().calculate(targetVel, error);
    }

    public double coast(double tangentialVel, double theta, PathProgress pathProgress, double constrainedVelocity) {
        double targetCoastDecel = coastingDecelerationConstraint.get().radius(theta);
        double coastVelNeededToStopInTime = Math.sqrt(config.coastDownToVelocity.get() * config.coastDownToVelocity.get() + 2 * Math.abs(targetCoastDecel) * pathProgress.distanceRemaining);

        double zeroPowerCoastFinalVelSquared = tangentialVel * tangentialVel + 2 * maxAchievableDeceleration.radius(theta) * pathProgress.distanceRemaining;
        double zeroPowerCoastFinalVel = Math.signum(zeroPowerCoastFinalVelSquared) * Math.sqrt(Math.abs(zeroPowerCoastFinalVelSquared));
        double targetVel = Math.min(coastVelNeededToStopInTime, constrainedVelocity);

        double velocityMomentumCannotProvide = Math.max(0, (config.coastDownToVelocity.get() - zeroPowerCoastFinalVel));
        double feedforwardVelocity = Math.min(constrainedVelocity, velocityMomentumCannotProvide);

        double error = Math.max(0, targetVel - tangentialVel);
        return config.coastController.get().calculate(feedforwardVelocity, error);
    }

    private Vector2D getBrakeDisplacement(double v, double theta) {
        Vector2D unit = Vector2D.unit(theta);
        Vector2D quadraticTerm = unit.hadamardProduct(unit).transform(config.quadraticBrakeCoefficients.get()).times(v * Math.abs(v));
        Vector2D linearTerm = unit.transform(config.linearBrakeCoefficients.get()).times(v);
        return quadraticTerm.plus(linearTerm);
    }
}