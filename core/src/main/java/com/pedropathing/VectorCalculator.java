package com.pedropathing;

import com.pedropathing.control.ControllerManager;
import com.pedropathing.control.Controller;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.QuadraticDampedController;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;

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
    
    private ControllerManager controllerManager;
    
    private Controller translationalPIDF;
    private Controller headingPIDF;
    private Controller drivePIDF;

    public VectorCalculator(FollowerConstants constants) {
        this.constants = constants;
        this.controllerManager = constants.controllerManager;
        
        drivePIDF = this.controllerManager.createDrive();
        translationalPIDF = this.controllerManager.createTranslational();
        headingPIDF = this.controllerManager.createHeading();

        updateConstants();
    }
    
    public void updateConstants() {
        controllerManager.updateCoefficients();

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
        controllerManager.reset();
        
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
    
    public double getTotalDistanceRemaining() {
        if (currentPath.isAtParametricEnd()) {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(currentPose.getX() - currentPath.getLastControlPoint().getX(), currentPose.getY() - currentPath.getLastControlPoint().getY());
            return currentPath.getEndTangent().dot(offset);
        }
        
        if (!followingPathChain) {
            return currentPath.getDistanceRemaining();
        }
        
        PathChain.DecelerationType type = currentPathChain.getDecelerationType();
        if (type != PathChain.DecelerationType.GLOBAL) {
            return currentPath.getDistanceRemaining();
        }
        
        double remainingLength = 0;
        
        if (chainIndex < currentPathChain.size()) {
            for (int i = chainIndex + 1; i < currentPathChain.size(); i++) {
                remainingLength += currentPathChain.getPath(i).length();
            }
        }
        
        return remainingLength + currentPath.getDistanceRemaining();
    }
    
    /**
     * Prevents the robot from applying too much power in the opposite direction of the
     * robot's momentum. This prevents voltage drops and doesn't hurt braking performance
     * that much since reversing the direction of wheels with just -0.001 power behaves
     * identical to -0.3 at high speeds due to EMF braking.
     *
     * @author Jacob Ophoven - 18535, Frozen Code
     */
    private double clampReversePower(double power, double directionOfMotion) {
        boolean isOpposingMotion = directionOfMotion * power < 0;
        
        if (!isOpposingMotion) {
            return power;
        }
        
        double clampedPower;
        if (power < 0) {
            clampedPower = Math.max(power, -0.2);
        } else {
            clampedPower = Math.min(power, 0.2);
        }
        return clampedPower;
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
        
        Vector tangent = currentPath.getClosestPointTangentVector().normalize();
        
        // Should also be an option for a singular path, not just path chain
        if (followingPathChain && currentPathChain.getDecelerationType() == PathChain.DecelerationType.FAST_BRAKE) {
            double totalDistanceRemaining = getTotalDistanceRemaining();
            
            // 20 is the max braking distance possible (prevents the robot from
            // skipping part of tight paths)
            boolean isNearEnd = totalDistanceRemaining <= 20;
            double brakingPower = translationalPIDF.run(totalDistanceRemaining);
            boolean isDecelerating = brakingPower < maxPowerScaling;
            if (isNearEnd && isDecelerating) {
                // if the controller is quadratic and
                // deceleration is set to none then exit the path here instead of
                // waiting until overshooting the path.
                double tangentialVelocity = velocity.dot(tangent);
                driveVector = new Vector(clampReversePower(MathFunctions.clamp(brakingPower
                    , -maxPowerScaling, maxPowerScaling), tangentialVelocity),
                                         tangent.getTheta());
                return driveVector.copy();
            }
        }
        
        if (driveError == -1) return new Vector(maxPowerScaling, currentPath.getClosestPointTangentVector().getTheta());
        
        driveVector = new Vector(MathFunctions.clamp(drivePIDF.run(driveError), -maxPowerScaling, maxPowerScaling), tangent.getTheta());
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
        
        headingVector = new Vector(MathFunctions.clamp(headingPIDF.run(headingError), -maxPowerScaling, maxPowerScaling), currentPose.getHeading());
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
            translationalVector = translationalVector.minus(new Vector(
                translationalVector.dot(
                    currentPath.getClosestPointTangentVector().normalize()),
                currentPath.getClosestPointTangentVector().getTheta()));
        }
        
        translationalVector.setMagnitude(MathFunctions.clamp(translationalPIDF.run(translationalVector.getMagnitude()), 0, maxPowerScaling));

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
     */
    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric) {
        teleopDriveValues[0] = MathFunctions.clamp(forwardDrive, -1, 1);
        teleopDriveValues[1] = MathFunctions.clamp(lateralDrive, -1, 1);
        teleopDriveValues[2] = MathFunctions.clamp(heading, -1, 1);
        teleopDriveVector.setOrthogonalComponents(teleopDriveValues[0], teleopDriveValues[1]);
        teleopDriveVector.setMagnitude(MathFunctions.clamp(teleopDriveVector.getMagnitude(), 0, 1));

        if (robotCentric) {
            teleopDriveVector.rotateVector(currentPose.getHeading());
        }

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
        constants.setCoefficientsDrivePIDF(drivePIDFCoefficients);
    }

    public void setSecondaryDrivePIDFCoefficients(FilteredPIDFCoefficients secondaryDrivePIDFCoefficients) {
        constants.secondaryDrivePIDFCoefficients(secondaryDrivePIDFCoefficients);
    }

    public void setHeadingPIDFCoefficients(PIDFCoefficients headingPIDFCoefficients) {
        constants.setCoefficientsHeadingPIDF(headingPIDFCoefficients);
    }

    public void setSecondaryHeadingPIDFCoefficients(PIDFCoefficients secondaryHeadingPIDFCoefficients) {
        constants.setCoefficientsSecondaryHeadingPIDF(secondaryHeadingPIDFCoefficients);
    }

    public void setTranslationalPIDFCoefficients(PIDFCoefficients translationalPIDFCoefficients) {
        constants.setCoefficientsTranslationalPIDF(translationalPIDFCoefficients);
    }

    public void setSecondaryTranslationalPIDFCoefficients(PIDFCoefficients secondaryTranslationalPIDFCoefficients) {
        constants.secondaryTranslationalPIDFCoefficients(secondaryTranslationalPIDFCoefficients);
    }

    public void setConstants(FollowerConstants constants) {
        this.constants = constants;
    }

    public String debugString() {
        return "Drive Vector: " + getDriveVector().toString() + "\n" +
                "Heading Vector: " + getHeadingVector().toString() + "\n" +
                "Translational Vector: " + getTranslationalVector().toString() + "\n" +
                "Centripetal Vector: " + getCentripetalVector().toString() + "\n" +
                "Corrective Vector: " + getCorrectiveVector().toString() + "\n" +
                "Teleop Drive Vector: " + getTeleopDriveVector().toString() + "\n" +
                "Teleop Heading Vector: " + getTeleopHeadingVector().toString();
    }
}
