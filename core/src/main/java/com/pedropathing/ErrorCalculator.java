package com.pedropathing;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.math.Kinematics;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.math.MathFunctions;

import java.util.Arrays;

/** This is the ErrorCalculator.
 * It is in charge of taking the Poses and Velocity produced by the PoseTracker and determining and returning the errors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 */
public class ErrorCalculator {
    private FollowerConstants constants;
    private KalmanFilter driveKalmanFilter;
    private Pose closestPose, currentPose;
    private Path currentPath;
    private PathChain currentPathChain;
    private boolean followingPathChain;
    private double[] driveErrors;
    private int chainIndex;
    private double rawDriveError, previousRawDriveError, headingError, xVelocity, yVelocity;
    private Double driveError;
    private Vector velocityVector = new Vector();
    private double headingGoal;
    
    public ErrorCalculator(FollowerConstants constants) {
        this.constants = constants;
        
        KalmanFilterParameters driveKalmanFilterParameters = new KalmanFilterParameters(
                6,
                1);

        driveKalmanFilter = new KalmanFilter(driveKalmanFilterParameters);

    }

    public void update(Pose currentPose, Path currentPath, PathChain currentPathChain, boolean followingPathChain, Pose closestPose, Vector velocity, int chainIndex, double xMovement, double yMovement, double headingGoal) {
        this.currentPose = currentPose;
        this.velocityVector = velocity;
        this.currentPath = currentPath;
        this.closestPose = closestPose;
        this.currentPathChain = currentPathChain;
        this.followingPathChain = followingPathChain;
        this.chainIndex = chainIndex;
        this.xVelocity = xMovement;
        this.yVelocity = yMovement;
        this.headingGoal = headingGoal;
        driveError = null;
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
        if (currentPath == null) {
            return 0;
        }

        headingError = MathFunctions.getTurnDirection(currentPose.getHeading(), headingGoal) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), headingGoal);
        return headingError;
    }

    /**
     * This returns the velocity the robot needs to be at to make it to the end of the Path
     * at some specified deceleration (well technically just some negative acceleration).
     *
     * @return returns the projected velocity.
     */
    private double getDriveVelocityError(double distanceToGoal, boolean deceleration, boolean maxVelocity) {
        if (currentPath == null) {
            return 0;
        }

        if (!deceleration && !maxVelocity) {
            return -1;
        }

        Vector tangent = currentPath.getClosestPointTangentVector().normalize();
        Vector distanceToGoalVector = tangent.times(distanceToGoal);
        Vector velocity = velocityVector.projectOnto(tangent);
        Vector targetVelocity = maxVelocity ? tangent.times(currentPath.getMaxVelocity()) : new Vector();
        Vector targetAcceleration = currentPath.getMaxAcceleration() > 0 ? tangent.times(currentPath.getMaxAcceleration()) : new Vector();

        Vector forwardHeadingVector = new Vector(1.0, currentPose.getHeading());
        double forwardVelocity = forwardHeadingVector.dot(velocity);
        double forwardDistanceToGoal = forwardHeadingVector.dot(distanceToGoalVector);
        double forwardVelocityGoal = maxVelocity ? forwardHeadingVector.dot(targetVelocity) : 0;
        double forwardAccelerationMaximum = currentPath.getMaxAcceleration() > 0 ? forwardHeadingVector.dot(targetAcceleration) : 0;
        double forwardAcceleration = currentPath.getMaxAcceleration() < 0 ? constants.forwardZeroPowerAcceleration :
                Math.min(forwardAccelerationMaximum, constants.forwardZeroPowerAcceleration);

        if (deceleration && !maxVelocity) {
            forwardVelocityGoal = Kinematics.getVelocityToStopWithDeceleration(
                    forwardDistanceToGoal,
                    forwardAcceleration
                            * (currentPath.getBrakingStrength() * 4)
            );
        } else if (deceleration) {
            forwardVelocityGoal = Math.min(Kinematics.getVelocityToStopWithDeceleration(
                    forwardDistanceToGoal,
                    forwardAcceleration
                            * (currentPath.getBrakingStrength() * 4)
            ), forwardVelocityGoal);
        }

        double forwardVelocityZeroPowerDecay = forwardVelocity -
            Kinematics.getFinalVelocityAtDistance(
                forwardVelocity,
                forwardAcceleration,
                forwardDistanceToGoal
            );

        Vector lateralHeadingVector = new Vector(1.0, currentPose.getHeading() - Math.PI / 2);
        double lateralVelocity = lateralHeadingVector.dot(velocity);
        double lateralDistanceToGoal = lateralHeadingVector.dot(distanceToGoalVector);
        double lateralVelocityGoal = maxVelocity ? lateralHeadingVector.dot(targetVelocity) : 0;
        double lateralAccelerationMaximum = currentPath.getMaxAcceleration() > 0 ? lateralHeadingVector.dot(targetAcceleration) : 0;
        double lateralAcceleration = currentPath.getMaxAcceleration() < 0 ? constants.lateralZeroPowerAcceleration :
                Math.min(lateralAccelerationMaximum, constants.lateralZeroPowerAcceleration);

        if (deceleration && !maxVelocity) {
            lateralVelocityGoal = Kinematics.getVelocityToStopWithDeceleration(
                    lateralDistanceToGoal,
                    lateralAcceleration
                            * (currentPath.getBrakingStrength() * 4)
            );
        } else if (deceleration) {
            lateralVelocityGoal = Math.min(Kinematics.getVelocityToStopWithDeceleration(
                    lateralDistanceToGoal,
                    lateralAcceleration
                            * (currentPath.getBrakingStrength() * 4)
            ), lateralVelocityGoal);
        }
        double lateralVelocityZeroPowerDecay = lateralVelocity -
            Kinematics.getFinalVelocityAtDistance(
                lateralVelocity,
                lateralAcceleration,
                lateralDistanceToGoal
            );

        Vector forwardVelocityError = new Vector(forwardVelocityGoal - forwardVelocityZeroPowerDecay - forwardVelocity, forwardHeadingVector.getTheta());
        Vector lateralVelocityError = new Vector(lateralVelocityGoal - lateralVelocityZeroPowerDecay - lateralVelocity, lateralHeadingVector.getTheta());
        Vector velocityErrorVector = forwardVelocityError.plus(lateralVelocityError);

        previousRawDriveError = rawDriveError;
        rawDriveError = velocityErrorVector.getMagnitude() * Math.signum(velocityErrorVector.dot(tangent));

        double projection = Kinematics.predictNextLoopVelocity(driveErrors[1], driveErrors[0]);

        driveKalmanFilter.update(rawDriveError - previousRawDriveError, projection);

        for (int i = 0; i < driveErrors.length - 1; i++) {
            driveErrors[i] = driveErrors[i + 1];
        }

        driveErrors[1] = driveKalmanFilter.getState();

        return driveKalmanFilter.getState();
    }

    public double getDriveError() {
        if (driveError != null) return driveError;

        double distanceToGoal;

        if (currentPath == null) {
            return 0;
        }

        if (!currentPath.isAtParametricEnd()) {
            if (followingPathChain) {
                PathChain.DecelerationType type = currentPathChain.getDecelerationType();
                if (type == PathChain.DecelerationType.GLOBAL) {
                    double remainingLength = 0;

                    if (chainIndex < currentPathChain.size() - 1) {
                        for (int i = chainIndex + 1; i < currentPathChain.size(); i++) {
                            remainingLength += currentPathChain.getPath(i).length();
                        }
                    }

                    distanceToGoal = remainingLength + currentPath.getDistanceRemaining();

                    Vector tangent = currentPath.getClosestPointTangentVector().normalize();
                    Vector forwardTheoreticalHeadingVector = new Vector(1.0, headingGoal);

                    double stoppingDistance = Kinematics.getStoppingDistance(
                            yVelocity + (xVelocity - yVelocity) * forwardTheoreticalHeadingVector.dot(tangent),
                            tangent.times(currentPath.getMaxAcceleration()).dot(forwardTheoreticalHeadingVector)
                    );
                    if (distanceToGoal >= stoppingDistance * currentPath.getBrakingStartMultiplier()) {
                        if (currentPath.getMaxVelocity() <= 0) return -1;
                        driveError = getDriveVelocityError(distanceToGoal, false, true);
                        return driveError;
                    }
                } else if ((type == PathChain.DecelerationType.LAST_PATH && chainIndex < currentPathChain.size() - 1) || type == PathChain.DecelerationType.NONE) {
                    if (currentPath.getMaxVelocity() <= 0) return -1;

                    double remainingLength = 0;

                    if (chainIndex < currentPathChain.size() - 1) {
                        for (int i = chainIndex + 1; i < currentPathChain.size(); i++) {
                            remainingLength += currentPathChain.getPath(i).length();
                        }
                    }

                    distanceToGoal = remainingLength + currentPath.getDistanceRemaining();
                    driveError = getDriveVelocityError(distanceToGoal, false, true);
                    return driveError;
                } else {
                    distanceToGoal = currentPath.getDistanceRemaining();
                }
            } else {
                distanceToGoal = currentPath.getDistanceRemaining();
            }
        } else {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(currentPose.getX() - currentPath.getLastControlPoint().getX(), currentPose.getY() - currentPath.getLastControlPoint().getY());
            distanceToGoal = currentPath.getEndTangent().dot(offset);
        }

        driveError = getDriveVelocityError(distanceToGoal, true, currentPath.getMaxVelocity() > 0);

        return driveError;
    }

    public double getRawDriveError() {
        return rawDriveError;
    }

    public double[] getDriveErrors() {
        return driveErrors;
    }

    public void breakFollowing() {
        driveError = 0.0;
        headingError = 0;
        rawDriveError = 0;
        previousRawDriveError = 0;
        driveErrors = new double[2];
        Arrays.fill(driveErrors, 0);
        driveKalmanFilter.reset();
    }

    public void setConstants(FollowerConstants constants) {
        this.constants = constants;
    }

    public String debugString() {
        return "Current Pose: " + currentPose.toString() + "\n" +
               "Closest Pose: " + closestPose.toString() + "\n" +
               "Current Path: " + (currentPath != null ? currentPath.toString() : "null") + "\n" + "Following Path Chain: " + followingPathChain + "\n" +
               "Chain Index: " + chainIndex + "\n" +
               "Drive Error: " + getDriveError() + "\n" +
               "Heading Error: " + getHeadingError() + "\n" +
               "Raw Drive Error: " + getRawDriveError();
    }
}
