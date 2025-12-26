package com.pedropathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;
import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.PIDFController;

import java.util.ArrayList;

/** This is the VectorCalculator.
 * It is in charge of taking the errors produced by the ErrorCalculator and determining and returning drive + corrective vectors
 *
 * @author Baron Henderson - 20077 The Indubitables
 */
public class VectorCalculator {
    private FollowerConstants constants;

    private Path currentPath;
    private PathChain currentPathChain;
    private Pose currentPose, closestPose;
    private double headingError, driveError;
    private double headingGoal;

    private ArrayList<Vector> velocities = new ArrayList<>();
    private ArrayList<Vector> accelerations = new ArrayList<>();
    private Vector velocity = new Vector();

    private Vector averageVelocity, averagePreviousVelocity, averageAcceleration;
    private Vector secondaryTranslationalIntegralVector, translationalIntegralVector;
    private Vector teleopDriveVector, teleopHeadingVector;

    public Vector driveVector, headingVector, translationalVector, centripetalVector, correctiveVector, translationalError;

    private double previousSecondaryTranslationalIntegral;
    private double previousTranslationalIntegral;
    public static double drivePIDFSwitch, headingPIDFSwitch, translationalPIDFSwitch;
    public static boolean useSecondaryDrivePID, useSecondaryHeadingPID, useSecondaryTranslationalPID;
    private double[] teleopDriveValues;

    private boolean useDrive = true, useHeading = true, useTranslational = true, useCentripetal = true, teleopDrive = false, followingPathChain = false;
    private double maxPowerScaling = 1.0, mass = 10.65;
    private boolean scaleDriveFeedforward;

    private int chainIndex;
    private double centripetalScaling;

    private PIDFController secondaryTranslationalPIDF;
    private PIDFController secondaryTranslationalIntegral;
    private PIDFController translationalPIDF;
    private PIDFController translationalIntegral;
    private PIDFController secondaryHeadingPIDF;

    private PIDFController headingPIDF;
    private FilteredPIDFController secondaryDrivePIDF, drivePIDF;

    public VectorCalculator(FollowerConstants constants) {
        this.constants = constants;
        drivePIDF = new FilteredPIDFController(constants.coefficientsDrivePIDF);
        secondaryDrivePIDF = new FilteredPIDFController(constants.coefficientsSecondaryDrivePIDF);
        headingPIDF = new PIDFController(constants.coefficientsHeadingPIDF);
        secondaryHeadingPIDF = new PIDFController(constants.coefficientsSecondaryHeadingPIDF);
        translationalPIDF = new PIDFController(constants.coefficientsTranslationalPIDF);
        secondaryTranslationalPIDF = new PIDFController(constants.coefficientsSecondaryTranslationalPIDF);
        translationalIntegral = new PIDFController(constants.integralTranslational);
        secondaryTranslationalIntegral = new PIDFController(constants.integralSecondaryTranslational);
        updateConstants();
    }
    
    public void updateConstants() {
        drivePIDF.setCoefficients(constants.coefficientsDrivePIDF);
        secondaryDrivePIDF.setCoefficients(constants.coefficientsSecondaryDrivePIDF);
        headingPIDF.setCoefficients(constants.coefficientsHeadingPIDF);
        secondaryHeadingPIDF.setCoefficients(constants.coefficientsSecondaryHeadingPIDF);
        translationalPIDF.setCoefficients(constants.coefficientsTranslationalPIDF);
        secondaryTranslationalPIDF.setCoefficients(constants.coefficientsSecondaryTranslationalPIDF);
        translationalIntegral.setCoefficients(constants.integralTranslational);
        secondaryTranslationalIntegral.setCoefficients(constants.integralSecondaryTranslational);
        drivePIDFSwitch = constants.drivePIDFSwitch;
        headingPIDFSwitch = constants.headingPIDFSwitch;
        translationalPIDFSwitch = constants.translationalPIDFSwitch;
        useSecondaryDrivePID = constants.useSecondaryDrivePIDF;
        useSecondaryHeadingPID = constants.useSecondaryHeadingPIDF;
        useSecondaryTranslationalPID = constants.useSecondaryTranslationalPIDF;
        mass = constants.mass;
    }

    public void update(boolean useDrive, boolean useHeading, boolean useTranslational, boolean useCentripetal, boolean teleopDrive, int chainIndex, double maxPowerScaling, boolean followingPathChain, double centripetalScaling, Pose currentPose, Pose closestPose, Vector velocity, Path currentPath, PathChain currentPathChain, double driveError, Vector translationalError, double headingError, double headingGoal) {
        updateConstants();

        this.useDrive = useDrive;
        this.useHeading = useHeading;
        this.useTranslational = useTranslational;
        this.useCentripetal = useCentripetal;
        this.teleopDrive = teleopDrive;
        this.maxPowerScaling = maxPowerScaling;
        this.chainIndex = chainIndex;
        this.followingPathChain = followingPathChain;
        this.centripetalScaling = centripetalScaling;
        this.currentPose = currentPose;
        this.closestPose = closestPose;
        this.velocity = velocity;
        this.currentPath = currentPath;
        this.currentPathChain = currentPathChain;
        this.driveError = driveError;
        this.translationalError = translationalError;
        this.headingError = headingError;
        this.headingGoal = headingGoal;

        if(teleopDrive)
            teleopUpdate();
    }
    
    public void breakFollowing() {
        secondaryDrivePIDF.reset();
        drivePIDF.reset();
        secondaryHeadingPIDF.reset();
        headingPIDF.reset();
        secondaryTranslationalPIDF.reset();
        secondaryTranslationalIntegral.reset();
        translationalPIDF.reset();
        translationalIntegral.reset();
        
        secondaryTranslationalIntegralVector = new Vector();
        translationalIntegralVector = new Vector();
        driveVector = new Vector();
        headingVector = new Vector();
        translationalVector = new Vector();
        centripetalVector = new Vector();
        correctiveVector = new Vector();

        int AVERAGED_VELOCITY_SAMPLE_NUMBER = 8;
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
        velocities.add(velocity);
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

        if (driveError == -1) return new Vector(maxPowerScaling, currentPath.getClosestPointTangentVector().getTheta());

        Vector tangent = currentPath.getClosestPointTangentVector().normalize();

        if (Math.abs(driveError) < drivePIDFSwitch && useSecondaryDrivePID) {
            secondaryDrivePIDF.updateFeedForwardInput(Math.signum(driveError));
            secondaryDrivePIDF.updateError(driveError);
            driveVector = new Vector(MathFunctions.clamp(secondaryDrivePIDF.run(), -maxPowerScaling, maxPowerScaling), tangent.getTheta());
            return driveVector.copy();
        }

        drivePIDF.updateFeedForwardInput(Math.signum(driveError));
        drivePIDF.updateError(driveError);
        driveVector = new Vector(MathFunctions.clamp(drivePIDF.run(), -maxPowerScaling, maxPowerScaling), tangent.getTheta());
        return driveVector.copy();
    }

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
            secondaryHeadingPIDF.updateFeedForwardInput(MathFunctions.getTurnDirection(currentPose.getHeading(), headingGoal));
            secondaryHeadingPIDF.updateError(headingError);
            headingVector = new Vector(MathFunctions.clamp(secondaryHeadingPIDF.run(), -maxPowerScaling, maxPowerScaling), currentPose.getHeading());
            return headingVector.copy();
        }
        headingPIDF.updateFeedForwardInput(MathFunctions.getTurnDirection(currentPose.getHeading(), headingGoal));
        headingPIDF.updateError(headingError);
        headingVector = new Vector(MathFunctions.clamp(headingPIDF.run(), -maxPowerScaling, maxPowerScaling), currentPose.getHeading());
        return headingVector.copy();
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
        Vector corrective = centripetal.plus(translational);

        if (corrective.getMagnitude() > maxPowerScaling) {
            return centripetal.plus(translational.times(MathFunctions.findNormalizingScaling(centripetal, translational, maxPowerScaling)));
        }

        correctiveVector = corrective.copy();

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
        Vector translationalVector = translationalError.copy();

        if (!(currentPath.isAtParametricEnd() || currentPath.isAtParametricStart())) {
            translationalVector = translationalVector.minus(new Vector(translationalVector.dot(currentPath.getClosestPointTangentVector().normalize()), currentPath.getClosestPointTangentVector().getTheta()));

            secondaryTranslationalIntegralVector = secondaryTranslationalIntegralVector.minus(new Vector(secondaryTranslationalIntegralVector.dot(currentPath.getClosestPointTangentVector().normalize()), currentPath.getClosestPointTangentVector().getTheta()));
            translationalIntegralVector = translationalIntegralVector.minus(new Vector(translationalIntegralVector.dot( currentPath.getClosestPointTangentVector().normalize()), currentPath.getClosestPointTangentVector().getTheta()));
        }

        if (currentPose.distanceFrom(closestPose) < translationalPIDFSwitch && useSecondaryTranslationalPID) {
            secondaryTranslationalIntegral.updateError(translationalVector.getMagnitude());
            secondaryTranslationalIntegralVector = secondaryTranslationalIntegralVector.plus(new Vector(secondaryTranslationalIntegral.run() - previousSecondaryTranslationalIntegral, translationalVector.getTheta()));
            previousSecondaryTranslationalIntegral = secondaryTranslationalIntegral.run();

            secondaryTranslationalPIDF.updateFeedForwardInput(1);
            secondaryTranslationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(secondaryTranslationalPIDF.run());
            translationalVector = translationalVector.plus(secondaryTranslationalIntegralVector);
        } else {
            translationalIntegral.updateError(translationalVector.getMagnitude());
            translationalIntegralVector = translationalIntegralVector.plus(new Vector(translationalIntegral.run() - previousTranslationalIntegral, translationalVector.getTheta()));
            previousTranslationalIntegral = translationalIntegral.run();

            translationalPIDF.updateFeedForwardInput(1);
            translationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(translationalPIDF.run());
            translationalVector = translationalVector.plus(translationalIntegralVector);
        }

        translationalVector.setMagnitude(MathFunctions.clamp(translationalVector.getMagnitude(), 0, maxPowerScaling));

        this.translationalVector = translationalVector.copy();

        return translationalVector;
    }

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
        centripetalVector = new Vector(MathFunctions.clamp(centripetalScaling * mass * Math.pow(velocity.dot(currentPath.getClosestPointTangentVector().normalize()), 2) * curvature, -maxPowerScaling, maxPowerScaling), currentPath.getClosestPointTangentVector().getTheta() + Math.PI / 2 * Math.signum(currentPath.getClosestPointNormalVector().getTheta()));
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
     * @param headingOffset sets an offset for the heading based either robot centric or field centric movement
     */
    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric, double headingOffset) {
        teleopDriveValues[0] = MathFunctions.clamp(forwardDrive, -1, 1);
        teleopDriveValues[1] = MathFunctions.clamp(lateralDrive, -1, 1);
        teleopDriveValues[2] = MathFunctions.clamp(heading, -1, 1);
        teleopDriveVector.setOrthogonalComponents(teleopDriveValues[0], teleopDriveValues[1]);
        teleopDriveVector.setMagnitude(MathFunctions.clamp(teleopDriveVector.getMagnitude(), 0, 1));

        if (robotCentric) {
            teleopDriveVector.rotateVector(currentPose.getHeading());
        }

        teleopDriveVector.rotateVector(headingOffset);

        teleopHeadingVector.setComponents(teleopDriveValues[2], currentPose.getHeading());
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

    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric) {
        setTeleOpMovementVectors(forwardDrive, lateralDrive, heading, robotCentric, 0);
    }

    /**
     * This calculates an averaged approximate velocity and acceleration. This is used for a
     * real-time correction of centripetal force, which is used in teleop.
     */
    public void calculateAveragedVelocityAndAcceleration() {
        averageVelocity = new Vector();
        averagePreviousVelocity = new Vector();

        for (int i = 0; i < velocities.size() / 2; i++) {
            averageVelocity = averageVelocity.plus(velocities.get(i));
        }
        averageVelocity = averageVelocity.times(1.0 / ((double) velocities.size() / 2));

        for (int i = velocities.size() / 2; i < velocities.size(); i++) {
            averagePreviousVelocity = averagePreviousVelocity.plus(velocities.get(i));
        }
        averagePreviousVelocity = averagePreviousVelocity.times(1.0 / ((double) velocities.size() / 2));

        accelerations.add(averageVelocity.minus(averagePreviousVelocity));
        accelerations.remove(accelerations.size() - 1);

        averageAcceleration = new Vector();

        for (int i = 0; i < accelerations.size(); i++) {
            averageAcceleration = averageAcceleration.plus(accelerations.get(i));
        }
        averageAcceleration = averageAcceleration.times(1.0 / accelerations.size());
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

    public void setDrivePIDFCoefficients(FilteredPIDFCoefficients drivePIDFCoefficients) {
        this.drivePIDF.setCoefficients(drivePIDFCoefficients);
    }

    public void setSecondaryDrivePIDFCoefficients(FilteredPIDFCoefficients secondaryDrivePIDFCoefficients) {
        this.secondaryDrivePIDF.setCoefficients(secondaryDrivePIDFCoefficients);
    }

    public void setHeadingPIDFCoefficients(PIDFCoefficients headingPIDFCoefficients) {
        this.headingPIDF.setCoefficients(headingPIDFCoefficients);
    }

    public void setSecondaryHeadingPIDFCoefficients(PIDFCoefficients secondaryHeadingPIDFCoefficients) {
        this.secondaryHeadingPIDF.setCoefficients(secondaryHeadingPIDFCoefficients);
    }

    public void setTranslationalPIDFCoefficients(PIDFCoefficients translationalPIDFCoefficients) {
        translationalPIDF.setCoefficients(translationalPIDFCoefficients);
    }

    public void setSecondaryTranslationalPIDFCoefficients(PIDFCoefficients secondaryTranslationalPIDFCoefficients) {
        this.secondaryTranslationalPIDF.setCoefficients(secondaryTranslationalPIDFCoefficients);
    }

    public void setConstants(FollowerConstants constants) {
        this.constants = constants;
    }

    public String debugString() {
        return "Drive Vector: " + driveVector.toString() + "\n" +
                "Heading Vector: " + headingVector.toString() + "\n" +
                "Translational Vector: " + translationalVector.toString() + "\n" +
                "Centripetal Vector: " + centripetalVector.toString() + "\n" +
                "Corrective Vector: " + correctiveVector.toString() + "\n" +
                "Teleop Drive Vector: " + teleopDriveVector.toString() + "\n" +
                "Teleop Heading Vector: " + teleopHeadingVector.toString();
    }
}