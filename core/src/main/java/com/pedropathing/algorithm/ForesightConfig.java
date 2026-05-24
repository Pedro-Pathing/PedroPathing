package com.pedropathing.algorithm;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;
import com.pedropathing.config.Memoize;
import com.pedropathing.config.Validator;
import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDCoefficients;
import com.pedropathing.math.Ellipse2D;
import com.pedropathing.math.Matrix;
import com.pedropathing.utils.Pair;

public final class ForesightConfig {
    public final ConfigVar<Controller> headingController = ConfigVar.of(Controller.pid(new PIDCoefficients(1.5, 0, 0.1)));
    // TODO test iZone, decay, and maxI to prevent integral wind-up and have zero-steady state error

    public final ConfigVar<Controller> translationalController = ConfigVar.of(Controller.pid(new PIDCoefficients(0.3, 0, 0)));

    public final ConfigVar<Controller> brakeController = ConfigVar.of(
            Controller.pid(new PIDCoefficients(0.025, 0, 0))
                    .plus(Controller.dynamicFeedforward(0.015))
                    .plus(Controller.staticFeedforward(0.05)));
    public final ConfigVar<Controller> coastController = ConfigVar.of(
            Controller.pid(new PIDCoefficients(0.025, 0, 0))
                    .plus(Controller.dynamicFeedforward(0.015))
                    .plus(Controller.staticFeedforward(0.05)));

    /** Centripetal force to power scaling. */
    public final ConfigVar<Double> centripetalScaling = ConfigVar.of(0.005, Validator.nonnegative());

    /**
     * The maximum amount of power the robot can apply in the opposite direction of momentum. Default is 0.2. Too high of a value might burn out the control hub and too low of a value might not be able to stop quickly after back-emf is overcome.
     */
    public final ConfigVar<Double> maxBrakingPower = ConfigVar.of(0.2, Validator.positive());

    /** The max acceleration the robot can travel along paths. */
    public final ConfigVar<Double> maxAcceleration = ConfigVar.of(Double.POSITIVE_INFINITY, Validator.positive());

    /** The max speed the robot can travel along paths. */
    public final ConfigVar<Double> maxVelocity = ConfigVar.of(Double.POSITIVE_INFINITY, Validator.positive());

    /**
     * Scale factor for the natural coasting deceleration constraint.
     * 1.0 uses the natural deceleration, while infinity removes the limit.
     */
    public final ConfigVar<Double> coastingConstraintScale = ConfigVar.of(Double.POSITIVE_INFINITY, Validator.positive());

    /**
     * How much overshooting is allowed when braking. A value of 1 means no bias, while a value greater than 1 means the controller will overshoot the target, and a value lower than 1 means the controller will undershoot the target.
     * <p>
     * Useful if you do not need to fully brake due to an obstacle that can slow you down.
     * Lower number is helpful if you need to ensure you do not overshoot the target.
     */
    public final ConfigVar<Double> brakingOvershootBias = ConfigVar.of(1.0, Validator.positive());

    public final ConfigVar<Double> velocityToBrakeTo = ConfigVar.of(0.0, Validator.nonnegative());

    /**
     * The velocity the robot coasts down to before it starts braking. Does nothing if the coastingConstraintScale is infinity.
     */
    public final ConfigVar<Double> coastDownToVelocity = ConfigVar.of(0.0, Validator.nonnegative());

    /**
     * Heading error where forward acceleration reaches zero.
     * Higher values prioritize path speed over heading accuracy.
     */
    public final ConfigVar<Double> headingDeviationTolerance =
            ConfigVar.of(Math.toRadians(45), Validator.positive());

    /**
     * Lateral deviation where forward acceleration reaches zero.
     * Higher values prioritize path speed over path accuracy.
     */
    public final ConfigVar<Double> lateralDeviationTolerance =
            ConfigVar.of(2.5, Validator.positive());

    /** Whether the robot brakes at the end of the path or not. */
    public final ConfigVar<Boolean> shouldBrakeAtEnd = ConfigVar.of(true);

    public final ConfigVar<Matrix> linearBrakeCoefficients = ConfigVar.required();
    public final ConfigVar<Matrix> quadraticBrakeCoefficients = ConfigVar.required();

    /**
     * Clamps the maximum velocity of the dynamic motion profile. Purely to avoid unnecessarily large numbers for debugging and graph visualization purposes.
     */
    public final ConfigVar<Ellipse2D> maxAchievableVelocity = ConfigVar.of(Ellipse2D.fromAxes(80.0, 65.0));

    /**
     * The natural deceleration of the robot when no power is applied.
     */
    public final ConfigVar<Ellipse2D> naturalDeceleration = ConfigVar.of(Ellipse2D.fromAxes(30.0, 30.0));

    /**
     * The distance the controller will stop commanding power to correct for path deviations.
     */
    public final ConfigVar<Double> minCorrectionDistance = ConfigVar.of(1e-3);

    public final Memoize<Pair<Ellipse2D, Double>, Ellipse2D> coastingDecelerationConstraint = Memoize.memo(
            () -> Pair.of(naturalDeceleration.get(), coastingConstraintScale.get()),
            p -> Ellipse2D.fromAxes(
                    -Math.abs(p.first().getMajorAxis()) * p.second(),
                    -Math.abs(p.first().getMinorAxis()) * p.second()
            )
    );

    public ForesightConfig(Configuration<ForesightConfig> config) {
        config.configure(this);
    }
}