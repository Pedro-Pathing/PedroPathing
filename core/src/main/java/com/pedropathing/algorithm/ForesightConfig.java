/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.algorithm;

import static com.pedropathing.config.Validator.nonnegative;
import static com.pedropathing.config.Validator.positive;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

public final class ForesightConfig {
    public final ConfigVar<Double> headingProportional = ConfigVar.of(1.5, nonnegative());
    public final ConfigVar<Double> headingDerivative = ConfigVar.of(0.1, nonnegative());
    public final ConfigVar<Double> headingStatic = ConfigVar.of(0.01, nonnegative());

    public final ConfigVar<Double> forwardTranslationalProportional = ConfigVar.of(0.3, nonnegative());
    public final ConfigVar<Double> forwardTranslationalStatic = ConfigVar.of(0.015, nonnegative());
    public final ConfigVar<Double> strafeTranslationalProportional = ConfigVar.of(0.3, nonnegative());
    public final ConfigVar<Double> strafeTranslationalStatic = ConfigVar.of(0.015, nonnegative());

    public final ConfigVar<Double> brakeProportional = ConfigVar.of(0.025, nonnegative());
    public final ConfigVar<Double> brakeStatic = ConfigVar.of(0.015, nonnegative());

    public final ConfigVar<Double> coastProportional = ConfigVar.of(0.025, nonnegative());
    public final ConfigVar<Double> coastStatic = ConfigVar.of(0.05, nonnegative());
    public final ConfigVar<Double> coastFeedforward = ConfigVar.of(0.015, nonnegative());

    /**
     * This scales the translational error correction power when holding.
     */
    public final ConfigVar<Double> holdPointTranslationalScaling = ConfigVar.of(0.45, nonnegative());

    /**
     * This scales the heading error correction power when holding.
     */
    public final ConfigVar<Double> holdPointHeadingScaling = ConfigVar.of(0.35, nonnegative());

    /**
     * Centripetal force to power scaling.
     */
    public final ConfigVar<Vector2D> centripetalGains = ConfigVar.of(Vector2D.cartesian(0.0065, 0.0065));

    /**
     * The maximum amount of power the robot can apply in the opposite direction of momentum. Default is 0.2. Too high of a value might burn out the control hub and too low of a value might not be able to stop quickly after back-emf is overcome.
     */
    public final ConfigVar<Double> maxBrakingPower = ConfigVar.of(0.2, positive());

    public final ConfigVar<Double> maxAccelerationConstraint = ConfigVar.of(Constraint.NONE, positive());
    public final ConfigVar<Double> maxVelocityConstraint = ConfigVar.of(Constraint.NONE, positive());
    public final ConfigVar<Double> maxDecelerationConstraint = ConfigVar.of(Constraint.NONE, positive());

    /**
     * Set the maxVelocityConstraint to a fraction of the maxAchievableVelocity.
     */
    public void setPathSpeed(double speed) {
        maxVelocityConstraint.set(maxAchievableForwardVelocity.get() * speed);
    }

    public static class Constraint {
        public static double NONE = Double.POSITIVE_INFINITY;
    }

    /**
     * How much overshooting is allowed when braking. A value of 1 means no bias, while a value greater than 1 means the controller will overshoot the target, and a value lower than 1 means the controller will undershoot the target.
     * <p>
     * Useful if you do not need to fully brake due to an obstacle that can slow you down.
     * Lower number is helpful if you need to ensure you do not overshoot the target.
     */
    public final ConfigVar<Double> brakeAggression = ConfigVar.of(1.0, positive());

    /**
     * The velocity the robot coasts down to before it starts braking. Does nothing if the coastingConstraintScale is infinity.
     */
    public final ConfigVar<Double> coastDownToVelocity = ConfigVar.of(0.0, nonnegative());

    public final ConfigVar<Double> headingDeviationTolerance = ConfigVar.of(Math.toRadians(11.25), positive());
    public final ConfigVar<Double> translationalDeviationTolerance = ConfigVar.of(2.5, positive());
    public final ConfigVar<Boolean> brakeAtEnd = ConfigVar.of(true);

    public final ConfigVar<Double> headingDriveRatio = ConfigVar.of(0.5, nonnegative());

    public final ConfigVar<Matrix> linearBrakeCoefficients = ConfigVar.required();
    public final ConfigVar<Matrix> quadraticBrakeCoefficients = ConfigVar.required();
    public final ConfigVar<Vector2D> headingBrakeCoefficients = ConfigVar.required();

    public final ConfigVar<Boolean> cosineScale = ConfigVar.of(true);
    public final ConfigVar<Boolean> turnBeforeDriving = ConfigVar.of(false);

    /**
     * Maximum achievable speed that the robot can move forward/backward at, in units per second.
     */
    public final ConfigVar<Double> maxAchievableForwardVelocity = ConfigVar.required(positive());

    /**
     * Maximum achievable speed that the robot can move laterally, in units per second.
     */
    public final ConfigVar<Double> maxAchievableStrafeVelocity = ConfigVar.required(positive());

    /**
     * Maximum achievable magnitude that the robot can decelerate forward/backward at, in units per second^2.
     */
    public final ConfigVar<Double> naturalForwardDeceleration = ConfigVar.required(positive());

    /**
     * Maximum achievable magnitude that the robot can decelerate laterally, in units per second^2.
     */
    public final ConfigVar<Double> naturalStrafeDeceleration = ConfigVar.required(positive());

    /**
     * The distance the controller will stop commanding power to correct for path deviations.
     */
    public final ConfigVar<Double> minCorrectionDistance = ConfigVar.of(1e-3);

    public final ConfigVar<Double> parametricTConstraint = ConfigVar.of(0.025, positive());
    public final ConfigVar<Double> headingConstraint = ConfigVar.of(0.007, positive());
    public final ConfigVar<Double> translationalConstraint = ConfigVar.of(0.1, positive());
    public final ConfigVar<Double> velocityConstraint = ConfigVar.of(0.1, positive());
    public final ConfigVar<Double> timeoutConstraint = ConfigVar.of(100.0, nonnegative());

    public ForesightConfig(Configuration<ForesightConfig> config) {
        config.configure(this);
    }
}
