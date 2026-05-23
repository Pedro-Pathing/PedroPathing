package com.pedropathing.utils;

import lombok.experimental.UtilityClass;

@UtilityClass
public class Utils {
    public static class Control {
        /**
         * Calculates the remaining magnitude on a unit circle after subtracting A component.
         */
        public static double getRemainingMagnitude(double totalMagnitude, double usedMagnitude) {
            return Math.sqrt(
                    Math.max(
                            0.0,
                            totalMagnitude * totalMagnitude - usedMagnitude * usedMagnitude
                    )
            );
        }

        /**
         * Allocates power to a control component while respecting a total power budget.
         */
        public static double allocatePower(double requested, double budget) {
            return Math.copySign(
                    Math.min(Math.abs(requested), budget),
                    requested
            );
        }

        /**
         * Scales the control output using a cosine function to avoid continuing when deviating far from the target.
         */
        public static double cosineScale(double error, double falloffRadius) {
            double clamped = Math.min(Math.abs(error) * ((Math.PI / 2) / falloffRadius), Math.PI / 2);
            return Math.cos(clamped);
        }

        /**
         * Clamps the braking power to a maximum value when it is in the opposite direction of motion. This prevents burnouts and low voltage spikes.
         *
         * @param directionOfMotion +1 or -1
         * @param maxBrakingPower   positive
         */
        public static double clampBrakingPower(double power, double directionOfMotion, double maxBrakingPower) {
            if (directionOfMotion * power >= 0) {
                return power;
            }
            return Math.copySign(Math.min(Math.abs(power), maxBrakingPower), power);
        }

        /**
         * Scales all values proportionally so none exceed a magnitude of 1.0
         */
        public static void desaturate(double[] powers) {
            double max = 1.0;

            for (double power : powers) {
                max = Math.max(max, Math.abs(power));
            }

            if (max > 1.0) {
                double scale = 1 / max;
                for (int i = 0; i < powers.length; i++) {
                    powers[i] *= scale;
                }
            }
        }
    }

    public static class Angle {
        /**
         * This normalizes an angle to be between 0 and 2 pi radians, inclusive.
         * <p>
         * IMPORTANT NOTE: This method operates in radians.
         *
         * @param angleRadians the angle to be normalized.
         * @return returns the normalized angle.
         */
        public static double normalize(double angleRadians) {
            double angle = angleRadians % (2*Math.PI);
            if (angle < 0) {
                return angle + 2*Math.PI;
            }
            return angle;
        }

        /**
         * This normalizes an angle to be between -PI and PI radians.
         * <p>
         * IMPORTANT NOTE: This method operates in radians.
         *
         * @param angleRadians the angle to be normalized.
         * @return returns the normalized angle between -PI (inclusive) and PI (exclusive).
         */
        public static double normalizeSigned(double angleRadians) {
            double angle = normalize(angleRadians);
            if (angle >= Math.PI) {
                return angle - 2*Math.PI;
            }
            return angle;
        }

        /**
         * This returns the smallest angle between two angles. This operates in radians.
         *
         * @param one one of the angles.
         * @param two the other one.
         * @return returns the smallest angle.
         */
        public static double smallestDifference(double one, double two) {
            return Math.min(normalize(one - two), normalize(two - one));
        }

        /**
         * This gets the direction to turn between a start heading and an end heading. Positive is left
         * and negative is right. This operates in radians.
         *
         * @return returns the turn direction.
         */
        public static double turnDirection(double startHeading, double endHeading) {
            if (normalize(endHeading - startHeading) >= 0 && normalize(endHeading - startHeading) <= Math.PI) {
                return 1; // counter clock wise
            }
            return -1; // clock wise
        }
    }

    public Pair<Double, Double> solveQuadratic(double a, double b, double c) {
        double sqrtD = java.lang.Math.sqrt(b * b - 4 * a * c);
        double q = -0.5 * (b + java.lang.Math.copySign(sqrtD, b));
        return Pair.of(q / a, c / q);
    }

    public double clamp(double num, double lower, double upper) {
        return java.lang.Math.max(lower, java.lang.Math.min(num, upper));
    }
}