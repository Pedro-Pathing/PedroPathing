package com.pedropathing.follower;

import math.Vector;

public abstract class Drivetrain {
    protected Vector[] vectors;
    protected double maxPowerScaling;

    /**
     * This takes in vectors for corrective power, heading power, and pathing power and outputs
     * an Array of four doubles, one for each wheel's motor power.
     *
     * IMPORTANT NOTE: all vector inputs are clamped between 0 and 1 inclusive in magnitude.
     *
     * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
     *                        translational power Vector to correct onto the Bezier curve the Follower
     *                        is following.
     * @param headingPower this Vector points in the direction of the robot's current heading, and
     *                     the magnitude tells the robot how much it should turn and in which
     *                     direction.
     * @param pathingPower this Vector points in the direction the robot needs to go to continue along
     *                     the Path.
     * @param robotHeading this is the current heading of the robot, which is used to calculate how
     *                     much power to allocate to each wheel.
     * @return this returns an Array of doubles with a length of 4, which contains the wheel powers.
     */
    public abstract double[] getDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading);

    /**
     * This takes in two Vectors, one static and one variable, and returns the scaling factor that,
     * when multiplied to the variable Vector, results in magnitude of the sum of the static Vector
     * and the scaled variable Vector being the max power scaling.
     *
     * IMPORTANT NOTE: I did not intend for this to be used for anything other than the method above
     * this one in this class, so there will be errors if you input Vectors of length greater than maxPowerScaling,
     * and it will scale up the variable Vector if the magnitude of the sum of the two input Vectors
     * isn't greater than maxPowerScaling. So, just don't use this elsewhere. There's gotta be a better way to do
     * whatever you're trying to do.
     *
     * I know that this is used outside of this class, however, I created this method so I get to
     * use it if I want to. Also, it's only used once outside of the DriveVectorScaler class, and
     * it's used to scale Vectors, as intended.
     *
     * @param staticVector the Vector that is held constant.
     * @param variableVector the Vector getting scaled to make the sum of the input Vectors have a
     *                       magnitude of maxPowerScaling.
     * @return returns the scaling factor for the variable Vector.
     */
    public abstract double findNormalizingScaling(Vector staticVector, Vector variableVector);
}
