package com.pedropathing.follower;

import static com.pedropathing.follower.old.FollowerConstants.drivePIDFFeedForward;
import static com.pedropathing.follower.old.FollowerConstants.drivePIDFSwitch;
import static com.pedropathing.follower.old.FollowerConstants.forwardZeroPowerAcceleration;
import static com.pedropathing.follower.old.FollowerConstants.headingPIDFFeedForward;
import static com.pedropathing.follower.old.FollowerConstants.headingPIDFSwitch;
import static com.pedropathing.follower.old.FollowerConstants.secondaryDrivePIDFFeedForward;
import static com.pedropathing.follower.old.FollowerConstants.secondaryHeadingPIDFFeedForward;
import static com.pedropathing.follower.old.FollowerConstants.secondaryTranslationalPIDFFeedForward;
import static com.pedropathing.follower.old.FollowerConstants.translationalPIDFFeedForward;
import static com.pedropathing.follower.old.FollowerConstants.translationalPIDFSwitch;
import static com.pedropathing.follower.old.FollowerConstants.useSecondaryDrivePID;
import static com.pedropathing.follower.old.FollowerConstants.useSecondaryHeadingPID;
import static com.pedropathing.follower.old.FollowerConstants.useSecondaryTranslationalPID;

import com.pedropathing.follower.old.FollowerConstants;
import com.pedropathing.localization.PoseUpdater;
import math.MathFunctions;
import com.pedropathing.paths.PathChain;
import math.Vector;
import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.PIDFController;

import java.util.ArrayList;

public class ErrorHandler {
    private PoseUpdater poseUpdater;
    private Drivetrain drivetrain;

    private ArrayList<Vector> velocities = new ArrayList<>();
    private ArrayList<Vector> accelerations = new ArrayList<>();

    private Vector averageVelocity;
    private Vector averagePreviousVelocity;
    private Vector averageAcceleration;
    private Vector secondaryTranslationalIntegralVector;
    private Vector translationalIntegralVector;
    private Vector teleopDriveVector;
    private Vector teleopHeadingVector;

    public Vector driveVector;
    public Vector headingVector;
    public Vector translationalVector;
    public Vector centripetalVector;
    public Vector correctiveVector;
    private final int AVERAGED_VELOCITY_SAMPLE_NUMBER = 8;;
    private double previousSecondaryTranslationalIntegral;
    private double previousTranslationalIntegral;
    private double[] teleopDriveValues;

    private boolean useDrive = true;
    private boolean useHeading = true;
    private boolean useTranslational = true;
    private boolean useCentripetal = true;
    private boolean teleopDrive = false;
    private boolean followingPathChain = false;
    private double maxPowerScaling = 1.0;

    private int chainIndex;
    private double centripetalScaling;

    private PIDFController secondaryTranslationalPIDF;
    private PIDFController secondaryTranslationalIntegral;
    private PIDFController translationalPIDF;
    private PIDFController translationalIntegral;
    private PIDFController secondaryHeadingPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController secondaryDrivePIDF;
    private FilteredPIDFController drivePIDF;


    public ErrorHandler(PoseUpdater poseUpdater, Drivetrain drivetrain) {
        this.poseUpdater = poseUpdater;
        this.drivetrain = drivetrain;
    }

    public void update(Follower follower) {
        this.useDrive = follower.getUseDrive();
        this.useHeading = follower.getUseHeading();
        this.useTranslational = follower.getUseTranslational();
        this.useCentripetal = follower.getUseCentripetal();
        this.teleopDrive = follower.getTeleopDrive();
        this.maxPowerScaling = follower.getMaxPowerScaling();

        this.chainIndex = follower.getChainIndex();

        this.followingPathChain = follower.getFollowingPathChain();
        this.centripetalScaling = follower.getCentripetalScaling();
        this.secondaryTranslationalPIDF = follower.getSecondaryTranslationalPIDF();
        this.secondaryTranslationalIntegral = follower.getSecondaryTranslationalIntegral();
        this.translationalPIDF = follower.getTranslationalPIDF();
        this.translationalIntegral = follower.getTranslationalIntegral();
        this.secondaryHeadingPIDF = follower.getSecondaryHeadingPIDF();
        this.headingPIDF = follower.getHeadingPIDF();
        this.secondaryDrivePIDF = follower.getSecondaryDrivePIDF();
        this.drivePIDF = follower.getDrivePIDF();
    }
    
    public void breakFollowing() {
        secondaryTranslationalIntegralVector = new Vector();
        translationalIntegralVector = new Vector();
        driveVector = new Vector();
        headingVector = new Vector();
        translationalVector = new Vector();
        centripetalVector = new Vector();
        correctiveVector = new Vector();

        for (int i = 0; i < AVERAGED_VELOCITY_SAMPLE_NUMBER; i++) {
            velocities.add(new Vector());
        }
        for (int i = 0; i < AVERAGED_VELOCITY_SAMPLE_NUMBER / 2; i++) {
            accelerations.add(new Vector());
        }

        calculateAveragedVelocityAndAcceleration();
        teleopDriveVector = new Vector();
        teleopHeadingVector = new Vector();
        previousSecondaryTranslationalIntegral = 0;
        previousTranslationalIntegral = 0;
        teleopDriveValues = new double[3];
    }

    /**
     * Do the teleop calculations
     */
    public void teleopUpdate() {
        velocities.add(poseUpdater.getVelocity());
        velocities.remove(velocities.get(velocities.size() - 1));

        calculateAveragedVelocityAndAcceleration();
    }

    /**
     * This returns a Vector in the direction the robot must go to move along the path. This Vector
     * takes into account the projected position of the robot to calculate how much power is needed.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the drive vector.
     */
    public Vector getDriveVector() {
        if (!useDrive) return new Vector();

        if (followingPathChain && ((chainIndex < currentPathChain.size() - 1 && currentPathChain.getDecelerationType() == PathChain.DecelerationType.LAST_PATH) || currentPathChain.getDecelerationType() == PathChain.DecelerationType.NONE)) {
            return new Vector(maxPowerScaling, currentPath.getClosestPointTangentVector().getTheta());
        }

        double distanceToGoal;
        if (!currentPath.isAtParametricEnd()) {
            if (followingPathChain && currentPathChain.getDecelerationType() == PathChain.DecelerationType.GLOBAL) {
                double remainingLength = 0;

                if (chainIndex < currentPathChain.size()) {
                    for (int i = chainIndex + 1; i<currentPathChain.size(); i++) {
                        remainingLength += currentPathChain.getPath(i).length();
                    }
                }

                distanceToGoal = remainingLength + currentPath.length() * (1 - currentPath.getClosestPointTValue());

                if (distanceToGoal >= Math.abs(currentPathChain.getDecelerationStartMultiplier() * 3/2 * Math.pow(FollowerConstants.xMovement, 2) / forwardZeroPowerAcceleration)) {
                    return new Vector(maxPowerScaling, currentPath.getClosestPointTangentVector().getTheta());
                }
            } else {
                distanceToGoal = currentPath.length() * (1 - currentPath.getClosestPointTValue());
            }
        } else {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(poseUpdater.getPose().getX() - currentPath.getLastControlPoint().getX(), poseUpdater.getPose().getY() - currentPath.getLastControlPoint().getY());
            distanceToGoal = MathFunctions.dotProduct(currentPath.getEndTangent(), offset);
        }

        driveError = getDriveVelocityError(distanceToGoal);

        if (Math.abs(driveError) < drivePIDFSwitch && useSecondaryDrivePID) {
            secondaryDrivePIDF.updateError(driveError);
            driveVector = new Vector(MathFunctions.clamp(secondaryDrivePIDF.runPIDF() + secondaryDrivePIDFFeedForward * MathFunctions.getSign(driveError), -maxPowerScaling, maxPowerScaling), currentPath.getClosestPointTangentVector().getTheta());
            return MathFunctions.copyVector(driveVector);
        }

        drivePIDF.updateError(driveError);
        driveVector = new Vector(MathFunctions.clamp(drivePIDF.runPIDF() + drivePIDFFeedForward * MathFunctions.getSign(driveError), -maxPowerScaling, maxPowerScaling), currentPath.getClosestPointTangentVector().getTheta());
        return MathFunctions.copyVector(driveVector);
    }

//    /**
//     * This returns the velocity the robot needs to be at to make it to the end of the Path
//     * at some specified deceleration (well technically just some negative acceleration).
//     *
//     * @return returns the projected velocity.
//     */
//    public double getDriveVelocityError(double distanceToGoal) {
//        Vector distanceToGoalVector = MathFunctions.scalarMultiplyVector(MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector()), distanceToGoal);
//        Vector velocity = new Vector(MathFunctions.dotProduct(poseUpdater.getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta());
//
//        Vector forwardHeadingVector = new Vector(1.0, poseUpdater.getPose().getHeading());
//
//        double forwardVelocity = MathFunctions.dotProduct(forwardHeadingVector, velocity);
//        double forwardDistanceToGoal = MathFunctions.dotProduct(forwardHeadingVector, distanceToGoalVector);
//        double forwardVelocityGoal = MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() * forwardZeroPowerAcceleration * (forwardDistanceToGoal <= 0 ? 1 : -1) * forwardDistanceToGoal));
//        double forwardVelocityZeroPowerDecay = forwardVelocity - MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(forwardVelocity, 2) + 2 * forwardZeroPowerAcceleration * Math.abs(forwardDistanceToGoal)));
//
//        Vector lateralHeadingVector = new Vector(1.0, poseUpdater.getPose().getHeading() - Math.PI / 2);
//        double lateralVelocity = MathFunctions.dotProduct(lateralHeadingVector, velocity);
//        double lateralDistanceToGoal = MathFunctions.dotProduct(lateralHeadingVector, distanceToGoalVector);
//
//        double lateralVelocityGoal = MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() * lateralZeroPowerAcceleration * (lateralDistanceToGoal <= 0 ? 1 : -1) * lateralDistanceToGoal));
//        double lateralVelocityZeroPowerDecay = lateralVelocity - MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(lateralVelocity, 2) + 2 * lateralZeroPowerAcceleration * Math.abs(lateralDistanceToGoal)));
//
//        Vector forwardVelocityError = new Vector(forwardVelocityGoal - forwardVelocityZeroPowerDecay - forwardVelocity, forwardHeadingVector.getTheta());
//        Vector lateralVelocityError = new Vector(lateralVelocityGoal - lateralVelocityZeroPowerDecay - lateralVelocity, lateralHeadingVector.getTheta());
//        Vector velocityErrorVector = MathFunctions.addVectors(forwardVelocityError, lateralVelocityError);
//
//        previousRawDriveError = rawDriveError;
//        rawDriveError = velocityErrorVector.getMagnitude() * MathFunctions.getSign(MathFunctions.dotProduct(velocityErrorVector, currentPath.getClosestPointTangentVector()));
//
//        double projection = 2 * driveErrors[1] - driveErrors[0];
//
//        driveKalmanFilter.update(rawDriveError - previousRawDriveError, projection);
//
//        for (int i = 0; i < driveErrors.length - 1; i++) {
//            driveErrors[i] = driveErrors[i + 1];
//        }
//        driveErrors[1] = driveKalmanFilter.getState();
//
//        return driveKalmanFilter.getState();
//    }
    /**
     * This returns a Vector in the direction of the robot that contains the heading correction
     * as its magnitude. Positive heading correction turns the robot counter-clockwise, and negative
     * heading correction values turn the robot clockwise. So basically, Pedro Pathing uses a right-
     * handed coordinate system.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the heading vector.
     */
    public Vector getHeadingVector() {
        if (!useHeading) return new Vector();
        if (Math.abs(headingError) < headingPIDFSwitch && useSecondaryHeadingPID) {
            secondaryHeadingPIDF.updateError(headingError);
            headingVector = new Vector(MathFunctions.clamp(secondaryHeadingPIDF.runPIDF() + secondaryHeadingPIDFFeedForward * MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()), -maxPowerScaling, maxPowerScaling), poseUpdater.getPose().getHeading());
            return MathFunctions.copyVector(headingVector);
        }
        headingPIDF.updateError(headingError);
        headingVector = new Vector(MathFunctions.clamp(headingPIDF.runPIDF() + headingPIDFFeedForward * MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()), -maxPowerScaling, maxPowerScaling), poseUpdater.getPose().getHeading());
        return MathFunctions.copyVector(headingVector);
    }

    /**
     * This returns a combined Vector in the direction the robot must go to correct both translational
     * error as well as centripetal force.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the corrective vector.
     */
    public Vector getCorrectiveVector() {
        Vector centripetal = getCentripetalForceCorrection();
        Vector translational = getTranslationalCorrection();
        Vector corrective = MathFunctions.addVectors(centripetal, translational);

        if (corrective.getMagnitude() > maxPowerScaling) {
            return MathFunctions.addVectors(centripetal, MathFunctions.scalarMultiplyVector(translational, drivetrain.findNormalizingScaling(centripetal, translational)));
        }

        correctiveVector = MathFunctions.copyVector(corrective);

        return corrective;
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only translational
     * error.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the translational correction vector.
     */
    public Vector getTranslationalCorrection() {
        if (!useTranslational) return new Vector();
        Vector translationalVector = new Vector();
        double x = closestPose.getX() - poseUpdater.getPose().getX();
        double y = closestPose.getY() - poseUpdater.getPose().getY();
        translationalVector.setOrthogonalComponents(x, y);

        if (!(currentPath.isAtParametricEnd() || currentPath.isAtParametricStart())) {
            translationalVector = MathFunctions.subtractVectors(translationalVector, new Vector(MathFunctions.dotProduct(translationalVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));

            secondaryTranslationalIntegralVector = MathFunctions.subtractVectors(secondaryTranslationalIntegralVector, new Vector(MathFunctions.dotProduct(secondaryTranslationalIntegralVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));
            translationalIntegralVector = MathFunctions.subtractVectors(translationalIntegralVector, new Vector(MathFunctions.dotProduct(translationalIntegralVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));
        }

        if (MathFunctions.distance(poseUpdater.getPose(), closestPose) < translationalPIDFSwitch && useSecondaryTranslationalPID) {
            secondaryTranslationalIntegral.updateError(translationalVector.getMagnitude());
            secondaryTranslationalIntegralVector = MathFunctions.addVectors(secondaryTranslationalIntegralVector, new Vector(secondaryTranslationalIntegral.runPIDF() - previousSecondaryTranslationalIntegral, translationalVector.getTheta()));
            previousSecondaryTranslationalIntegral = secondaryTranslationalIntegral.runPIDF();

            secondaryTranslationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(secondaryTranslationalPIDF.runPIDF() + secondaryTranslationalPIDFFeedForward);
            translationalVector = MathFunctions.addVectors(translationalVector, secondaryTranslationalIntegralVector);
        } else {
            translationalIntegral.updateError(translationalVector.getMagnitude());
            translationalIntegralVector = MathFunctions.addVectors(translationalIntegralVector, new Vector(translationalIntegral.runPIDF() - previousTranslationalIntegral, translationalVector.getTheta()));
            previousTranslationalIntegral = translationalIntegral.runPIDF();

            translationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(translationalPIDF.runPIDF() + translationalPIDFFeedForward);
            translationalVector = MathFunctions.addVectors(translationalVector, translationalIntegralVector);
        }

        translationalVector.setMagnitude(MathFunctions.clamp(translationalVector.getMagnitude(), 0, maxPowerScaling));

        this.translationalVector = MathFunctions.copyVector(translationalVector);

        return translationalVector;
    }

//    /**
//     * This returns the raw translational error, or how far off the closest point the robot is.
//     *
//     * @return This returns the raw translational error as a Vector.
//     */
//    public Vector getTranslationalError() {
//        Vector error = new Vector();
//        double x = closestPose.getX() - poseUpdater.getPose().getX();
//        double y = closestPose.getY() - poseUpdater.getPose().getY();
//        error.setOrthogonalComponents(x, y);
//        return error;
//    }

    /**
     * This returns a Vector in the direction the robot must go to account for only centripetal
     * force.
     * <p>
     * Note: This vector is clamped to be between [0, 1] in magnitude.
     *
     * @return returns the centripetal force correction vector.
     */
    public Vector getCentripetalForceCorrection() {
        if (!useCentripetal) return new Vector();
        double curvature;
        if (!teleopDrive) {
            curvature = currentPath.getClosestPointCurvature();
        } else {
            double yPrime = averageVelocity.getYComponent() / averageVelocity.getXComponent();
            double yDoublePrime = averageAcceleration.getYComponent() / averageVelocity.getXComponent();
            curvature = (yDoublePrime) / (Math.pow(Math.sqrt(1 + Math.pow(yPrime, 2)), 3));
        }
        if (Double.isNaN(curvature)) return new Vector();
        centripetalVector = new Vector(MathFunctions.clamp(centripetalScaling * FollowerConstants.mass * Math.pow(MathFunctions.dotProduct(poseUpdater.getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), 2) * curvature, -maxPowerScaling, maxPowerScaling), currentPath.getClosestPointTangentVector().getTheta() + Math.PI / 2 * MathFunctions.getSign(currentPath.getClosestPointNormalVector().getTheta()));
        return centripetalVector;
    }

    /**
     * This sets the teleop drive vectors.
     *
     * @param forwardDrive determines the forward drive vector for the robot in teleop. In field centric
     *                     movement, this is the x-axis.
     * @param lateralDrive determines the lateral drive vector for the robot in teleop. In field centric
     *                     movement, this is the y-axis.
     * @param heading determines the heading vector for the robot in teleop.
     * @param robotCentric sets if the movement will be field or robot centric
     */
    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric) {
        teleopDriveValues[0] = MathFunctions.clamp(forwardDrive, -1, 1);
        teleopDriveValues[1] = MathFunctions.clamp(lateralDrive, -1, 1);
        teleopDriveValues[2] = MathFunctions.clamp(heading, -1, 1);
        teleopDriveVector.setOrthogonalComponents(teleopDriveValues[0], teleopDriveValues[1]);
        teleopDriveVector.setMagnitude(MathFunctions.clamp(teleopDriveVector.getMagnitude(), 0, 1));

        if (robotCentric) {
            teleopDriveVector.rotateVector(poseUpdater.getPose().getHeading());
        }

        teleopHeadingVector.setComponents(teleopDriveValues[2], poseUpdater.getPose().getHeading());
    }

    /**
     * This sets the teleop drive vectors. This defaults to robot centric.
     *
     * @param forwardDrive determines the forward drive vector for the robot in teleop. In field centric
     *                     movement, this is the x-axis.
     * @param lateralDrive determines the lateral drive vector for the robot in teleop. In field centric
     *                     movement, this is the y-axis.
     * @param heading determines the heading vector for the robot in teleop.
     */
    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading) {
        setTeleOpMovementVectors(forwardDrive, lateralDrive, heading, true);
    }

    /**
     * This calculates an averaged approximate velocity and acceleration. This is used for a
     * real-time correction of centripetal force, which is used in teleop.
     */
    public void calculateAveragedVelocityAndAcceleration() {
        averageVelocity = new Vector();
        averagePreviousVelocity = new Vector();

        for (int i = 0; i < velocities.size() / 2; i++) {
            averageVelocity = MathFunctions.addVectors(averageVelocity, velocities.get(i));
        }
        averageVelocity = MathFunctions.scalarMultiplyVector(averageVelocity, 1.0 / ((double) velocities.size() / 2));

        for (int i = velocities.size() / 2; i < velocities.size(); i++) {
            averagePreviousVelocity = MathFunctions.addVectors(averagePreviousVelocity, velocities.get(i));
        }
        averagePreviousVelocity = MathFunctions.scalarMultiplyVector(averagePreviousVelocity, 1.0 / ((double) velocities.size() / 2));

        accelerations.add(MathFunctions.subtractVectors(averageVelocity, averagePreviousVelocity));
        accelerations.remove(accelerations.size() - 1);

        averageAcceleration = new Vector();

        for (int i = 0; i < accelerations.size(); i++) {
            averageAcceleration = MathFunctions.addVectors(averageAcceleration, accelerations.get(i));
        }
        averageAcceleration = MathFunctions.scalarMultiplyVector(averageAcceleration, 1.0 / accelerations.size());
    }

    public boolean isTeleopDrive() {
        return teleopDrive;
    }

    public Vector getCentripetalVector() {
        return centripetalVector;
    }

    public Vector getTranslationalVector() {
        return translationalVector;
    }

    public Vector getTeleopHeadingVector() {
        return teleopHeadingVector;
    }

    public Vector getTeleopDriveVector() {
        return teleopDriveVector;
    }

    public Vector getTranslationalIntegralVector() {
        return translationalIntegralVector;
    }

    public Vector getAverageAcceleration() {
        return averageAcceleration;
    }

    public Vector getSecondaryTranslationalIntegralVector() {
        return secondaryTranslationalIntegralVector;
    }

    public Vector getAveragePreviousVelocity() {
        return averagePreviousVelocity;
    }

    public Vector getAverageVelocity() {
        return averageVelocity;
    }

    public KalmanFilter getDriveKalmanFilter() {
        return driveKalmanFilter;
    }
    
}