package com.pedropathing.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import java.util.function.Supplier;

/**
 * A heading interpolator is a function that takes a path and returns the heading goal the robot
 * should be at a specific point on the path.
 * Note: these methods all use radians.
 *
 * <p>
 * Example usage:
 * <pre><code>
 * // Default: tangent to the path
 * .setHeadingInterpolation(HeadingInterpolator.tangent);
 * // Offset the tangent by 90 degrees
 * .setHeadingInterpolation(HeadingInterpolator.tangent.offset(90));
 * // Follow the path with the robot facing backwards
 * .setHeadingInterpolation(HeadingInterpolator.tangent.reverse());
 * // Follow the path while the robot is facing a point
 * .setHeadingInterpolation(HeadingInterpolator.facingPoint(5, 5));
 * // Make the robot have a constant heading of 45 degrees
 * .setHeadingInterpolation(HeadingInterpolator.constant(45));
 * // Make the robot transition from 0 degrees to 90 degrees over the path
 * .setHeadingInterpolation(HeadingInterpolator.linear(0, 90));
 *
 * .offset and .reverse methods can be applied to any HeadingInterpolator.
 * </code></pre>
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
@FunctionalInterface
public interface HeadingInterpolator {
    /**
     * This returns the heading interpolation for the PathPoint
     * @param closestPoint the PathPoint where the desired heading interpolation is to be found
     * @return the heading interpolation
     */
    double interpolate(PathPoint closestPoint);

    /**
     * This class allows chaining HeadingInterpolators. Each PiecewiseNode represents a HeadingInterpolation for a segment of the path, which can be used to create a piecewise heading interpolation.
     *
     * @author Havish Sripada - 12808 RevAmped Robotics
     */
    class PiecewiseNode {
        double initialTValue;
        double finalTValue;
        HeadingInterpolator interpolator;

        /**
         * Constructor for a piecewise heading node.
         * @param initialTValue first t-value where the heading interpolation is used inclusive
         * @param finalTValue final t-value where the heading interpolation is used inclusive
         * @param interpolator the heading interpolator to use on that interval
         */
        public PiecewiseNode(double initialTValue, double finalTValue, HeadingInterpolator interpolator) {
            this.initialTValue = initialTValue;
            this.finalTValue = finalTValue;
            this.interpolator = interpolator;
        }

        /**
         * Gets the initial t-value of the piecewise node.
         */
        public double getInitialTValue() {
            return initialTValue;
        }

        /**
         * Gets the final t-value of the piecewise node.
         */
        public double getFinalTValue() {
            return finalTValue;
        }

        /**
         * Gets the heading interpolator of the piecewise node.
         */
        public HeadingInterpolator getInterpolator() {
            return interpolator;
        }

        /**
         * The robot will transition from the start heading to the end heading from startT by endT.
         */
        public static PiecewiseNode linear(double startT, double endT, double startHeadingRad, double endHeadingRad) {
            return new PiecewiseNode(startT, endT, HeadingInterpolator.linear(startHeadingRad, endHeadingRad));
        }

        /**
         * The robot will transition from the start heading to the end heading from startT by endT.
         */
        public static PiecewiseNode reversedLinear(double startT, double endT, double startHeadingRad, double endHeadingRad) {
            return new PiecewiseNode(startT, endT, HeadingInterpolator.reversedLinear(startHeadingRad, endHeadingRad));
        }
    }

    static HeadingInterpolator lazy(Supplier<HeadingInterpolator> generator) {
        return new HeadingInterpolator() {
            private HeadingInterpolator inner;

            @Override
            public double interpolate(PathPoint closestPoint) {
                return inner.interpolate(closestPoint);
            }

            @Override
            public void init() {
                inner = generator.get();
                inner.init();
            }
        };
    }

    /**
     * Offsets the heading interpolator by a given amount.
     */
    default HeadingInterpolator offset(double offsetRad) {
        HeadingInterpolator outer = this;
        return new HeadingInterpolator() {
            @Override
            public double interpolate(PathPoint closestPoint) {
                return outer.interpolate(closestPoint) + MathFunctions.normalizeAngle(offsetRad);
            }

            @Override
            public void init() {
                outer.init();
            }
        };
    }
    
    /**
     * Reverses the direction of a heading interpolator.
     * <p>
     * Note: For facing a point this will make the robot face the opposite direction of the point.
     */
    default HeadingInterpolator reverse() {
        HeadingInterpolator outer = this;
        return new HeadingInterpolator() {
            @Override
            public double interpolate(PathPoint closestPoint) {
                return MathFunctions.normalizeAngle(outer.interpolate(closestPoint) + Math.PI);
            }

            @Override
            public void init() {
                outer.init();
            }
        };
    }
    
    /**
     * The robot will face the the direction of the path.
     */
    HeadingInterpolator tangent = closestPoint -> closestPoint.tangentVector.getTheta();
    
    /**
     * A constant heading along a path.
     */
    static HeadingInterpolator constant(double headingRad) {
        return path -> MathFunctions.normalizeAngle(headingRad);
    }
    
    /**
     * The robot will transition from the start heading to the end heading.
     */
    static HeadingInterpolator linear(double startHeadingRad, double endHeadingRad) {
        return linear(startHeadingRad, endHeadingRad, 1);
    }

    /**
     * The robot will transition from the start heading to the end heading.
     */
    static HeadingInterpolator reversedLinear(double startHeadingRad, double endHeadingRad) {
        return reversedLinear(startHeadingRad, endHeadingRad, 1);
    }

    /**
     * The robot will transition from the start heading to the end heading by endT.
     */
    static HeadingInterpolator linear(double startHeadingRad, double endHeadingRad, double endT) {
        startHeadingRad = MathFunctions.normalizeAngle(startHeadingRad);
        endHeadingRad = MathFunctions.normalizeAngle(endHeadingRad);
        double finalStartHeadingRad = startHeadingRad;
        double finalEndHeadingRad = endHeadingRad;

        return closestPoint -> {
            double clampedEndT = MathFunctions.clamp(endT, 0.0001, 1);
            double t = Math.min(closestPoint.tValue / clampedEndT, 1.0);
            double deltaHeading = MathFunctions.getTurnDirection(finalStartHeadingRad, finalEndHeadingRad) * MathFunctions.getSmallestAngleDifference(finalEndHeadingRad, finalStartHeadingRad);
            return MathFunctions.normalizeAngle(finalStartHeadingRad + deltaHeading * t);
        };
    }

    /**
     * The robot will transition from the start heading to the end heading by endT.
     */
    static HeadingInterpolator reversedLinear(double startHeadingRad, double endHeadingRad, double endT) {
        startHeadingRad = MathFunctions.normalizeAngle(startHeadingRad);
        endHeadingRad = MathFunctions.normalizeAngle(endHeadingRad);
        double finalStartHeadingRad = startHeadingRad;
        double finalEndHeadingRad = endHeadingRad;

        return closestPoint -> {
            double clampedEndT = MathFunctions.clamp(endT, 0.0001, 1);
            double t = Math.min(closestPoint.tValue / clampedEndT, 1.0);
            double deltaHeading = -MathFunctions.getTurnDirection(finalStartHeadingRad, finalEndHeadingRad) * Math.max(MathFunctions.normalizeAngle(finalEndHeadingRad - finalStartHeadingRad), MathFunctions.normalizeAngle(finalStartHeadingRad - finalEndHeadingRad));
            return MathFunctions.normalizeAngle(finalStartHeadingRad + deltaHeading * t);
        };
    }

    /**
     * The robot will always be facing the given point while following the path.
     */
    static HeadingInterpolator facingPoint(double x, double y) {
        return closestPoint -> MathFunctions.normalizeAngle(Math.atan2(
            y - closestPoint.pose.getY(),
            x - closestPoint.pose.getX()
        ));
    }

    /**
     * The robot will always be facing the given point while following the path.
     */
    static HeadingInterpolator facingPoint(Pose pose) {
        return facingPoint(pose.getX(), pose.getY());
    }

    /**
     * Define a custom piecewise interpolation across the path
     * @param nodes The nodes of the piecewise interpolation, make sure all t-values from [0,1] are covered
     */
    static HeadingInterpolator piecewise(PiecewiseNode... nodes) {
        return new HeadingInterpolator() {
            @Override
            public double interpolate(PathPoint closestPoint) {
                for (PiecewiseNode node : nodes) {
                    if (closestPoint.getTValue() >= node.getInitialTValue() && closestPoint.getTValue() <= node.getFinalTValue()) {
                        PathPoint scaledClosestPoint = new PathPoint(
                                MathFunctions.scale(
                                        closestPoint.getTValue(),
                                        node.getInitialTValue(),
                                        node.getFinalTValue(),
                                        0,
                                        1
                                ),
                                closestPoint.getPose(),
                                closestPoint.getTangentVector()
                        );
                        return node.getInterpolator().interpolate(scaledClosestPoint);
                    }
                }

                return MathFunctions.normalizeAngle(tangent.interpolate(closestPoint));
            }

            @Override
            public void init() {
                for (PiecewiseNode node : nodes) node.interpolator.init();
            }
        };
    }

    /**
     * A functional interface that can be used to provide a future double value.
     * This is useful for providing values that may not be known at the time of creating the
     * HeadingInterpolator, such as sensor readings or dynamic targets.
     */
    @FunctionalInterface
    interface FutureDouble {
        double get();
    }

    /**
     * Optional initialization method for implementations that need to set up state before being used.
     * This can be called once before the first call to interpolate.
     */
    default void init() {
        // Optional initialization method for implementations
    }

    /**
     * The robot will transition from the start heading to the end heading from startT by endT.
     * The start and/or end headings are provided as FutureDoubles, which are functions that return a double.
     * This allows for dynamic headings that can be determined at runtime.
     */
    static HeadingInterpolator linearFromPoint(FutureDouble startHeadingRad, FutureDouble endHeadingRad, double endT) {
        return lazy(() -> linear(startHeadingRad.get(), endHeadingRad.get(), endT));
    }

    /**
     * The robot will transition from the start heading to the end heading from startT by endT.
     * The start and/or end headings are provided as FutureDoubles, which are functions that return a double.
     * This allows for dynamic headings that can be determined at runtime.
     */
    static HeadingInterpolator linearFromPoint(FutureDouble startHeadingRad, double endHeadingRad, double endT) {
        return linearFromPoint(startHeadingRad, () -> endHeadingRad, endT);
    }

    /**
     * The robot will transition from the start heading to the end heading from startT by endT using reversed linear interpolation.
     * The start and/or end headings are provided as FutureDoubles, which are functions that return a double.
     * This allows for dynamic headings that can be determined at runtime.
     */
    static HeadingInterpolator reversedLinearFromPoint(FutureDouble startHeadingRad, double endHeadingRad, double endT) {
        return linearFromPoint(startHeadingRad, () -> endHeadingRad, endT);
    }

    /**
     * The robot will transition from the start heading to the end heading from startT by endT.
     * The start and/or end headings are provided as FutureDoubles, which are functions that return a double.
     * This allows for dynamic headings that can be determined at runtime.
     */
    static HeadingInterpolator linearFromPoint(double startHeadingRad, FutureDouble endHeadingRad, double endT) {
        return linearFromPoint(() -> startHeadingRad, endHeadingRad, endT);
    }

    /**
     * The robot will transition from the start heading to the end heading from startT by endT using reversed linear interpolation.
     * The start and/or end headings are provided as FutureDoubles, which are functions that return a double.
     * This allows for dynamic headings that can be determined at runtime.
     */
    static HeadingInterpolator reversedLinearFromPoint(double startHeadingRad, FutureDouble endHeadingRad, double endT) {
        return reversedLinearFromPoint(() -> startHeadingRad, endHeadingRad, endT);
    }

    /**
     * The robot will transition from the start heading to the end heading from startT by endT using reversed linear interpolation.
     * The start and/or end headings are provided as FutureDoubles, which are functions that return a double.
     * This allows for dynamic headings that can be determined at runtime.
     */
    static HeadingInterpolator reversedLinearFromPoint(FutureDouble startHeadingRad, FutureDouble endHeadingRad, double endT) {
        return lazy(() -> reversedLinear(startHeadingRad.get(), endHeadingRad.get(), endT));
    }
}