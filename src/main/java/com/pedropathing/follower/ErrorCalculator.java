package com.pedropathing.follower;

import static com.pedropathing.follower.old.FollowerConstants.forwardZeroPowerAcceleration;
import static com.pedropathing.follower.old.FollowerConstants.lateralZeroPowerAcceleration;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import math.Vector;
import math.MathFunctions;

import java.util.Arrays;

public class ErrorCalculator {

    // Static instance of the singleton
    private static ErrorCalculator instance;
    private KalmanFilter driveKalmanFilter;
    private Pose closestPose, currentPose;
    private Path currentPath;
    private PathChain currentPathChain;
    private double[] driveErrors;
    private double rawDriveError, previousRawDriveError, driveError, headingError;
    private Vector velocityVector = new Vector();

    // Private constructor to prevent instantiation
    private ErrorCalculator() {
        KalmanFilterParameters driveKalmanFilterParameters = new KalmanFilterParameters(
                6,
                1);

        driveKalmanFilter = new KalmanFilter(driveKalmanFilterParameters);
    }

    // Public method to provide access to the instance
    public static ErrorCalculator getInstance() {
        if (instance == null) {
            synchronized (ErrorCalculator.class) {
                if (instance == null) {
                    instance = new ErrorCalculator();
                }
            }
        }
        return instance;
    }

    public void update(Pose currentPose, Path currentPath, PathChain currentPathChain, Vector velocity) {
        this.currentPose = currentPose;
        this.velocityVector = velocity;
        this.currentPath = currentPath;
        this.closestPose = this.currentPath.getClosestPoint(currentPose, 10);
        this.currentPathChain = currentPathChain;
    }

    /**
     * This returns the raw translational error, or how far off the closest point the robot is.
     *
     * @return This returns the raw translational error as a Vector.
     */
    public Vector getTranslationalError() {
        Vector error = new Vector();
        double x = closestPose.getX() - currentPose.getX();
        double y = closestPose.getY() - currentPose.getY();
        error.setOrthogonalComponents(x, y);
        return error;
    }

    /**
     * This returns the raw heading error, or how far off the closest point the robot is.
     *
     * @return This returns the raw heading error as a double.
     */
    public double getHeadingError() {
        headingError = MathFunctions.getTurnDirection(currentPose.getHeading(), currentPath.getClosestPointHeadingGoal()) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), currentPath.getClosestPointHeadingGoal());
        return headingError;
    }

    /**
     * This returns the velocity the robot needs to be at to make it to the end of the Path
     * at some specified deceleration (well technically just some negative acceleration).
     *
     * @return returns the projected velocity.
     */
    public double getDriveVelocityError(double distanceToGoal) {
        Vector distanceToGoalVector = MathFunctions.scalarMultiplyVector(MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector()), distanceToGoal);
        Vector velocity = new Vector(MathFunctions.dotProduct(velocityVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta());

        Vector forwardHeadingVector = new Vector(1.0, currentPose.getHeading());

        double forwardVelocity = MathFunctions.dotProduct(forwardHeadingVector, velocity);
        double forwardDistanceToGoal = MathFunctions.dotProduct(forwardHeadingVector, distanceToGoalVector);
        double forwardVelocityGoal = MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() * forwardZeroPowerAcceleration * (forwardDistanceToGoal <= 0 ? 1 : -1) * forwardDistanceToGoal));
        double forwardVelocityZeroPowerDecay = forwardVelocity - MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(forwardVelocity, 2) + 2 * forwardZeroPowerAcceleration * Math.abs(forwardDistanceToGoal)));

        Vector lateralHeadingVector = new Vector(1.0, currentPose.getHeading() - Math.PI / 2);
        double lateralVelocity = MathFunctions.dotProduct(lateralHeadingVector, velocity);
        double lateralDistanceToGoal = MathFunctions.dotProduct(lateralHeadingVector, distanceToGoalVector);

        double lateralVelocityGoal = MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() * lateralZeroPowerAcceleration * (lateralDistanceToGoal <= 0 ? 1 : -1) * lateralDistanceToGoal));
        double lateralVelocityZeroPowerDecay = lateralVelocity - MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(lateralVelocity, 2) + 2 * lateralZeroPowerAcceleration * Math.abs(lateralDistanceToGoal)));

        Vector forwardVelocityError = new Vector(forwardVelocityGoal - forwardVelocityZeroPowerDecay - forwardVelocity, forwardHeadingVector.getTheta());
        Vector lateralVelocityError = new Vector(lateralVelocityGoal - lateralVelocityZeroPowerDecay - lateralVelocity, lateralHeadingVector.getTheta());
        Vector velocityErrorVector = MathFunctions.addVectors(forwardVelocityError, lateralVelocityError);

        previousRawDriveError = rawDriveError;
        rawDriveError = velocityErrorVector.getMagnitude() * MathFunctions.getSign(MathFunctions.dotProduct(velocityErrorVector, currentPath.getClosestPointTangentVector()));

        double projection = 2 * driveErrors[1] - driveErrors[0];

        driveKalmanFilter.update(rawDriveError - previousRawDriveError, projection);

        for (int i = 0; i < driveErrors.length - 1; i++) {
            driveErrors[i] = driveErrors[i + 1];
        }

        driveErrors[1] = driveKalmanFilter.getState();

        return driveKalmanFilter.getState();
    }

    public double getDriveError() {
        return driveError;
    }

    public double getRawDriveError() {
        return rawDriveError;
    }

    public double[] getDriveErrors() {
        return driveErrors;
    }

    public void breakFollowing() {
        driveError = 0;
        headingError = 0;
        rawDriveError = 0;
        previousRawDriveError = 0;
        driveErrors = new double[2];
        Arrays.fill(driveErrors, 0);
        driveKalmanFilter.reset();
    }
}
