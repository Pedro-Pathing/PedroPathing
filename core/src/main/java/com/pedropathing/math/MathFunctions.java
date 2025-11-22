package com.pedropathing.math;

/**
 * This is the MathFunctions class. This contains many useful math related methods that I use in
 * other classes to simplify code elsewhere.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/9/2024
 */
public class MathFunctions {

    /**
     * This clamps down a value to between the lower and upper bounds inclusive.
     *
     * @param num the number to be clamped.
     * @param lower the lower bound.
     * @param upper the upper bound.
     * @return returns the clamped number.
     */
    public static double clamp(double num, double lower, double upper) {
        if (num < lower) return lower;
        if (num > upper) return upper;
        return num;
    }

    /**
     * This normalizes an angle to be between 0 and 2 pi radians, inclusive.
     * <p>
     * IMPORTANT NOTE: This method operates in radians.
     *
     * @param angleRadians the angle to be normalized.
     * @return returns the normalized angle.
     */
    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (2*Math.PI);
        if (angle < 0) {
            return angle + 2*Math.PI;
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
    public static double getSmallestAngleDifference(double one, double two) {
        return Math.min(MathFunctions.normalizeAngle(one - two), MathFunctions.normalizeAngle(two - one));
    }

    /**
     * This gets the direction to turn between a start heading and an end heading. Positive is left
     * and negative is right. This operates in radians.
     *
     * @return returns the turn direction.
     */
    public static double getTurnDirection(double startHeading, double endHeading) {
        if (MathFunctions.normalizeAngle(endHeading - startHeading) >= 0 && MathFunctions.normalizeAngle(endHeading - startHeading) <= Math.PI) {
            return 1; // counter clock wise
        }
        return -1; // clock wise
    }

    /**
     * This returns whether a specified value is within a second specified value by plus/minus a
     * specified accuracy amount.
     *
     * @param one first number specified.
     * @param two Second number specified.
     * @param accuracy the level of accuracy specified.
     * @return returns if the two numbers are within the specified accuracy of each other.
     */
    public static boolean roughlyEquals(double one, double two, double accuracy) {
        return (one < two + accuracy && one > two - accuracy);
    }

    /**
     * This returns whether a specified number is within a second specified number by plus/minus 0.0001.
     *
     * @param one first number specified.
     * @param two second number specified.
     * @return returns if a specified number is within plus/minus 0.0001 of the second specified number.
     */
    public static boolean roughlyEquals(double one, double two) {
        return roughlyEquals(one, two, 0.0001);
    }

    /**
     * This takes in two Vectors, one static and one variable, and returns the scaling factor that,
     * when multiplied to the variable Vector, results in magnitude of the sum of the static Vector
     * and the scaled variable Vector being the max power scaling.
     *
     * IMPORTANT NOTE: There will be errors if you input Vectors of length greater than maxPowerScaling,
     * and it will scale up the variable Vector if the magnitude of the sum of the two input Vectors
     * isn't greater than maxPowerScaling.
     *
     * I know that this is used outside of this class, however, I created this method so I get to
     * use it if I want to.
     *
     * @param staticVector the Vector that is held constant.
     * @param variableVector the Vector getting scaled to make the sum of the input Vectors have a
     *                       magnitude of maxPowerScaling.
     * @return returns the scaling factor for the variable Vector.
     */
    public static double findNormalizingScaling(Vector staticVector, Vector variableVector, double maxPowerScaling) {
        double a = Math.pow(variableVector.getXComponent(), 2) + Math.pow(variableVector.getYComponent(), 2);
        double b = staticVector.getXComponent() * variableVector.getXComponent() + staticVector.getYComponent() * variableVector.getYComponent();
        double c = Math.pow(staticVector.getXComponent(), 2) + Math.pow(staticVector.getYComponent(), 2) - Math.pow(maxPowerScaling, 2);
        return (-b + Math.sqrt(Math.pow(b, 2) - a*c))/(a);
    }

    /**
         * Scales a number from one range to another.
         *
         * @param n  number to scale
         * @param x1 lower bound range of n
         * @param x2 upper bound range of n
         * @param y1 lower bound of scale
         * @param y2 upper bound of scale
         * @return a double scaled to a value between y1 and y2, inclusive
     */
    public static double scale(double n, double x1, double x2, double y1, double y2) {
        if (x2 - x1 == 0) {
            throw new IllegalArgumentException("Input range cannot be zero.");
        }
        return (n - x1) * (y2 - y1) / (x2 - x1) + y1;
    }
}
