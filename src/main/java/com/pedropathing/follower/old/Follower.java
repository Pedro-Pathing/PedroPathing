package com.pedropathing.follower.old;

import static com.pedropathing.follower.old.FollowerConstants.automaticHoldEnd;
import static com.pedropathing.follower.old.FollowerConstants.cacheInvalidateSeconds;
import static com.pedropathing.follower.old.FollowerConstants.drivePIDFFeedForward;
import static com.pedropathing.follower.old.FollowerConstants.drivePIDFSwitch;
import static com.pedropathing.follower.old.FollowerConstants.forwardZeroPowerAcceleration;
import static com.pedropathing.follower.old.FollowerConstants.headingPIDFFeedForward;
import static com.pedropathing.follower.old.FollowerConstants.headingPIDFSwitch;
import static com.pedropathing.follower.old.FollowerConstants.lateralZeroPowerAcceleration;
import static com.pedropathing.follower.old.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.old.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.old.FollowerConstants.nominalVoltage;
import static com.pedropathing.follower.old.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.old.FollowerConstants.rightRearMotorName;
import static com.pedropathing.follower.old.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.old.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.old.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.old.FollowerConstants.rightRearMotorDirection;
import static com.pedropathing.follower.old.FollowerConstants.secondaryDrivePIDFFeedForward;
import static com.pedropathing.follower.old.FollowerConstants.secondaryHeadingPIDFFeedForward;
import static com.pedropathing.follower.old.FollowerConstants.secondaryTranslationalPIDFFeedForward;
import static com.pedropathing.follower.old.FollowerConstants.translationalPIDFFeedForward;
import static com.pedropathing.follower.old.FollowerConstants.translationalPIDFSwitch;
import static com.pedropathing.follower.old.FollowerConstants.useSecondaryDrivePID;
import static com.pedropathing.follower.old.FollowerConstants.useSecondaryHeadingPID;
import static com.pedropathing.follower.old.FollowerConstants.useSecondaryTranslationalPID;
import static com.pedropathing.follower.old.FollowerConstants.useVoltageCompensationInAuto;
import static com.pedropathing.follower.old.FollowerConstants.useVoltageCompensationInTeleOp;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.pedropathing.control.CustomFilteredPIDFCoefficients;
import com.pedropathing.control.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.geometry.BezierPoint;
import math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.util.PathCallback;
import com.pedropathing.paths.PathChain;
import math.Vector;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This is the Follower class. It handles the actual following of the paths and all the on-the-fly
 * calculations that are relevant for movement.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/4/2024
 */
@Config
public class Follower {
    private HardwareMap hardwareMap;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    private DriveVectorScaler driveVectorScaler;

    public PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    private Pose closestPose;

    private Path currentPath;

    private PathChain currentPathChain;

    private int BEZIER_CURVE_SEARCH_LIMIT;
    private int AVERAGED_VELOCITY_SAMPLE_NUMBER;

    private int chainIndex;

    private long[] pathStartTimes;

    private boolean followingPathChain;
    private boolean holdingPosition;
    private boolean isBusy, isTurning;
    private boolean reachedParametricPathEnd;
    private boolean holdPositionAtEnd;
    private boolean teleopDrive;

    private double globalMaxPower = 1;
    private double previousSecondaryTranslationalIntegral;
    private double previousTranslationalIntegral;
    private double holdPointTranslationalScaling;
    private double holdPointHeadingScaling;
    public double driveError;
    public double headingError;

    private long reachedParametricPathEndTime;

    private double[] drivePowers;
    private double[] teleopDriveValues;

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

    private double centripetalScaling;

    private PIDFController secondaryTranslationalPIDF;
    private PIDFController secondaryTranslationalIntegral;
    private PIDFController translationalPIDF;
    private PIDFController translationalIntegral;
    private PIDFController secondaryHeadingPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController secondaryDrivePIDF;
    private FilteredPIDFController drivePIDF;

    private KalmanFilter driveKalmanFilter;
    private double[] driveErrors;
    private double rawDriveError;
    private double previousRawDriveError;
    private double turnHeadingErrorThreshold;

    public static boolean drawOnDashboard = true;
    public static boolean useTranslational = true;
    public static boolean useCentripetal = true;
    public static boolean useHeading = true;
    public static boolean useDrive = true;

    /*
     * Voltage Compensation
     * Credit to team 14343 Escape Velocity for the voltage code
     * Credit to team 23511 Seattle Solvers for implementing the voltage code into Follower.java
     */
    private boolean cached = false;

    private VoltageSensor voltageSensor;
    public double voltage = 0;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private boolean logDebug = true;

    private ElapsedTime zeroVelocityDetectedTimer;

    private Runnable resetFollowing = null;

    /**
     * This creates a new Follower given a HardwareMap.
     * @param hardwareMap HardwareMap required
     */
    public Follower(HardwareMap hardwareMap, Class<?> FConstants, Class<?> LConstants) {
        this.hardwareMap = hardwareMap;
        setupConstants(FConstants, LConstants);
        initialize();
    }

    /**
     * This creates a new Follower given a HardwareMap and a localizer.
     * @param hardwareMap HardwareMap required
     * @param localizer the localizer you wish to use
     */
    public Follower(HardwareMap hardwareMap, Localizer localizer, Class<?> FConstants, Class<?> LConstants) {
        this.hardwareMap = hardwareMap;
        setupConstants(FConstants, LConstants);
        initialize(localizer);
    }

    /**
     * Setup constants for the Follower.
     * @param FConstants the constants for the Follower
     * @param LConstants the constants for the Localizer
     */
    public void setupConstants(Class<?> FConstants, Class<?> LConstants) {
        Constants.setConstants(FConstants, LConstants);
        BEZIER_CURVE_SEARCH_LIMIT = FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT;
        AVERAGED_VELOCITY_SAMPLE_NUMBER = FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER;
        holdPointTranslationalScaling = FollowerConstants.holdPointTranslationalScaling;
        holdPointHeadingScaling = FollowerConstants.holdPointHeadingScaling;
        centripetalScaling = FollowerConstants.centripetalScaling;
        secondaryTranslationalPIDF = new PIDFController(FollowerConstants.secondaryTranslationalPIDFCoefficients);
        secondaryTranslationalIntegral = new PIDFController(FollowerConstants.secondaryTranslationalIntegral);
        translationalPIDF = new PIDFController(FollowerConstants.translationalPIDFCoefficients);
        translationalIntegral = new PIDFController(FollowerConstants.translationalIntegral);
        secondaryHeadingPIDF = new PIDFController(FollowerConstants.secondaryHeadingPIDFCoefficients);
        headingPIDF = new PIDFController(FollowerConstants.headingPIDFCoefficients);
        secondaryDrivePIDF = new FilteredPIDFController(FollowerConstants.secondaryDrivePIDFCoefficients);
        drivePIDF = new FilteredPIDFController(FollowerConstants.drivePIDFCoefficients);
        driveKalmanFilter = new KalmanFilter(FollowerConstants.driveKalmanFilterParameters);
        turnHeadingErrorThreshold = FollowerConstants.turnHeadingErrorThreshold;
    }

    /**
     * This initializes the follower.
     * In this, the DriveVectorScaler and PoseUpdater is instantiated, the drive motors are
     * initialized and their behavior is set, and the variables involved in approximating first and
     * second derivatives for teleop are set.
     */
    public void initialize() {
        poseUpdater = new PoseUpdater(hardwareMap);
        driveVectorScaler = new DriveVectorScaler(FollowerConstants.frontLeftVector);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageTimer.reset();

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        breakFollowing();
    }

    /**
     * This initializes the follower.
     * In this, the DriveVectorScaler and PoseUpdater is instantiated, the drive motors are
     * initialized and their behavior is set, and the variables involved in approximating first and
     * second derivatives for teleop are set.
     * @param localizer the localizer you wish to use
     */

    public void initialize(Localizer localizer) {
        poseUpdater = new PoseUpdater(hardwareMap, localizer);
        driveVectorScaler = new DriveVectorScaler(FollowerConstants.frontLeftVector);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageTimer.reset();

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        breakFollowing();
    }

    public void setCentripetalScaling(double set) {
        centripetalScaling = set;
    }

    /**
     * This sets the motors to the zero power behavior of brake.
     */
    private void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * This sets the motors to the zero power behavior of float.
     */
    private void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    /**
     * This sets the maximum power the motors are allowed to use.
     *
     * @param set This caps the motor power from [0, 1].
     */
    public void setMaxPower(double set) {
        globalMaxPower = set;
        driveVectorScaler.setMaxPowerScaling(set);
    }

    /**
     * This gets a Point from the current Path from a specified t-value.
     *
     * @return returns the Point.
     */
    public Pose getPointFromPath(double t) {
        if (currentPath != null) {
            return currentPath.getPoint(t);
        } else {
            return null;
        }
    }

    /**
     * This returns the current pose from the PoseUpdater.
     *
     * @return returns the pose
     */
    public Pose getPose() {
        return poseUpdater.getPose();
    }

    /**
     * This sets the current pose in the PoseUpdater without using offsets.
     *
     * @param pose The pose to set the current pose to.
     */
    public void setPose(Pose pose) {
        poseUpdater.setPose(pose);
    }

    /**
     * This returns the current velocity of the robot as a Vector.
     *
     * @return returns the current velocity as a Vector.
     */
    public Vector getVelocity() {
        return poseUpdater.getVelocity();
    }

    /**
     * This returns the current acceleration of the robot as a Vector.
     *
     * @return returns the current acceleration as a Vector.
     */
    public Vector getAcceleration() {
        return poseUpdater.getAcceleration();
    }

    /**
     * This returns the magnitude of the current velocity. For when you only need the magnitude.
     *
     * @return returns the magnitude of the current velocity.
     */
    public double getVelocityMagnitude() {
        return poseUpdater.getVelocity().getMagnitude();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param pose the pose to set the starting pose to.
     */
    public void setStartingPose(Pose pose) {
        poseUpdater.setStartingPose(pose);
    }

    /**
     * This sets the current pose, using offsets so no reset time delay. This is better than the
     * Road Runner reset, in general. Think of using offsets as setting trim in an aircraft. This can
     * be reset as well, so beware of using the resetOffset() method.
     *
     * @param set The pose to set the current pose to.
     */
    public void setCurrentPoseWithOffset(Pose set) {
        poseUpdater.setCurrentPoseWithOffset(set);
    }

    /**
     * This sets the offset for only the x position.
     *
     * @param xOffset This sets the offset.
     */
    public void setXOffset(double xOffset) {
        poseUpdater.setXOffset(xOffset);
    }

    /**
     * This sets the offset for only the y position.
     *
     * @param yOffset This sets the offset.
     */
    public void setYOffset(double yOffset) {
        poseUpdater.setYOffset(yOffset);
    }

    /**
     * This sets the offset for only the heading.
     *
     * @param headingOffset This sets the offset.
     */
    public void setHeadingOffset(double headingOffset) {
        poseUpdater.setHeadingOffset(headingOffset);
    }

    /**
     * This returns the x offset.
     *
     * @return returns the x offset.
     */
    public double getXOffset() {
        return poseUpdater.getXOffset();
    }

    /**
     * This returns the y offset.
     *
     * @return returns the y offset.
     */
    public double getYOffset() {
        return poseUpdater.getYOffset();
    }

    /**
     * This returns the heading offset.
     *
     * @return returns the heading offset.
     */
    public double getHeadingOffset() {
        return poseUpdater.getHeadingOffset();
    }

    /**
     * This resets all offsets set to the PoseUpdater. If you have reset your pose using the
     * setCurrentPoseUsingOffset(Pose set) method, then your pose will be returned to what the
     * PoseUpdater thinks your pose would be, not the pose you reset to.
     */
    public void resetOffset() {
        poseUpdater.resetOffset();
    }

    /**
     * This holds a Point.
     *
     * @param point   the Point to stay at.
     * @param heading the heading to face.
     */
    public void holdPoint(BezierPoint point, double heading) {
        breakFollowing();
        holdingPosition = true;
        isBusy = false;
        followingPathChain = false;
        currentPath = new Path(point);
        currentPath.setConstantHeadingInterpolation(heading);
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), 1);
    }


    /**
     * This holds a Point.
     *
     * @param pose the Point (as a Pose) to stay at.
     */
    public void holdPoint(Pose pose) {
        holdPoint(new BezierPoint(pose), pose.getHeading());
    }

    /**
     * This follows a Path.
     * This also makes the Follower hold the last Point on the Path.
     *
     * @param path the Path to follow.
     * @param holdEnd this makes the Follower hold the last Point on the Path.
     */
    public void followPath(Path path, boolean holdEnd) {
        driveVectorScaler.setMaxPowerScaling(globalMaxPower);
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        isBusy = true;
        followingPathChain = false;
        currentPath = path;
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
    }

    /**
     * This follows a Path.
     *
     * @param path the Path to follow.
     */
    public void followPath(Path path) {
        followPath(path, automaticHoldEnd);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     * This also makes the Follower hold the last Point on the PathChain.
     *
     * @param pathChain the PathChain to follow.
     * @param holdEnd this makes the Follower hold the last Point on the PathChain.
     */
    public void followPath(PathChain pathChain, boolean holdEnd) {
        followPath(pathChain, globalMaxPower, holdEnd);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     *
     * @param pathChain the PathChain to follow.
     */
    public void followPath(PathChain pathChain) {
        followPath(pathChain, automaticHoldEnd);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     * This also makes the Follower hold the last Point on the PathChain.
     *
     * @param pathChain the PathChain to follow.
     * @param maxPower the max power of the Follower for this path
     * @param holdEnd this makes the Follower hold the last Point on the PathChain.
     */
    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
        driveVectorScaler.setMaxPowerScaling(maxPower);
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        pathStartTimes = new long[pathChain.size()];
        pathStartTimes[0] = System.currentTimeMillis();
        isBusy = true;
        followingPathChain = true;
        chainIndex = 0;
        currentPathChain = pathChain;
        currentPath = pathChain.getPath(chainIndex);
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
        currentPathChain.resetCallbacks();
    }

    /**
     * Resumes pathing, can only be called after pausePathFollowing()
     */
    public void resumePathFollowing() {
        if (resetFollowing != null) {
            resetFollowing.run();
            resetFollowing = null;
            pathStartTimes = new long[currentPathChain.size()];
            pathStartTimes[0] = System.currentTimeMillis();
            isBusy = true;
            closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
        }
    }

    /**
     * Pauses pathing, can only be restarted with resumePathFollowing
     */
    public void pausePathFollowing(boolean holdPoint) {
        isBusy = false;

        boolean prevHoldEnd = holdPositionAtEnd;

        if (followingPathChain && currentPathChain != null) {
            PathChain lastChain = currentPathChain;
            int lastIndex = chainIndex;

            resetFollowing = () -> {
                followingPathChain = true;
                chainIndex = lastIndex;
                currentPathChain = lastChain;
                holdPositionAtEnd = prevHoldEnd;
                currentPath = currentPathChain.getPath(lastIndex);
            };
        } else if (currentPath != null) {
            Path lastPath = currentPath;

            resetFollowing = () -> {
                holdPositionAtEnd = prevHoldEnd;
                currentPath = lastPath;
            };
        }

        if (holdPoint) {
            holdPoint(getPose());
        } else {
            breakFollowing();
        }
    }

    /**
     * This starts teleop drive control.
     */
    public void startTeleopDrive() {
        breakFollowing();
        teleopDrive = true;

        if(FollowerConstants.useBrakeModeInTeleOp) {
            setMotorsToBrake();
        }
    }

    /**
     * Calls an update to the PoseUpdater, which updates the robot's current position estimate.
     */
    public void updatePose() {
        poseUpdater.update();

        if (drawOnDashboard) {
            dashboardPoseTracker.update();
        }
    }

    /**
     * This calls an update to the PoseUpdater, which updates the robot's current position estimate.
     * This also updates all the Follower's PIDFs, which updates the motor powers.
     */
    public void update() {
        updatePose();

        if (!teleopDrive) {
            if (currentPath != null) {
                if (holdingPosition) {
                    closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), 1);

                    drivePowers = driveVectorScaler.getDrivePowers(MathFunctions.scalarMultiplyVector(getTranslationalCorrection(), holdPointTranslationalScaling), MathFunctions.scalarMultiplyVector(getHeadingVector(), holdPointHeadingScaling), new Vector(), poseUpdater.getPose().getHeading());

                    for (int i = 0; i < motors.size(); i++) {
                        if (Math.abs(motors.get(i).getPower() - drivePowers[i]) > FollowerConstants.motorCachingThreshold) {
                            double voltageNormalized = getVoltageNormalized();

                            if (useVoltageCompensationInAuto) {
                                motors.get(i).setPower(drivePowers[i] * voltageNormalized);
                            } else {
                                motors.get(i).setPower(drivePowers[i]);
                            }
                        }
                    }

                    if(headingError < turnHeadingErrorThreshold && isTurning) {
                        isTurning = false;
                        isBusy = false;
                    }
                } else {
                    if (isBusy) {
                        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_SEARCH_LIMIT);

                        if (followingPathChain) updateCallbacks();

                        drivePowers = driveVectorScaler.getDrivePowers(getCorrectiveVector(), getHeadingVector(), getDriveVector(), poseUpdater.getPose().getHeading());

                        for (int i = 0; i < motors.size(); i++) {
                            if (Math.abs(motors.get(i).getPower() - drivePowers[i]) > FollowerConstants.motorCachingThreshold) {
                                double voltageNormalized = getVoltageNormalized();

                                if (useVoltageCompensationInAuto) {
                                    motors.get(i).setPower(drivePowers[i] * voltageNormalized);
                                } else {
                                    motors.get(i).setPower(drivePowers[i]);
                                }
                            }
                        }
                    }

                    // try to fix the robot stop near the end issue
                    // if robot is almost reach the end and velocity is close to zero
                    // then, break the following if other criteria meet
                    if (poseUpdater.getVelocity().getMagnitude() < 1.0 && currentPath.getClosestPointTValue() > 0.8
                            && zeroVelocityDetectedTimer == null && isBusy) {
                        zeroVelocityDetectedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

                        if (logDebug) {
                            Log.d("Follower_logger", "!!!! Robot stuck !!!!");
                            debugLog();
                        }
                    }

                    if (currentPath.isAtParametricEnd() ||
                            (zeroVelocityDetectedTimer != null && zeroVelocityDetectedTimer.milliseconds() > 500.0)) {
                        if (followingPathChain && chainIndex < currentPathChain.size() - 1) {

                            if (logDebug) {
                                Log.d("Follower_logger", "chainIndex: " + chainIndex + " | Pose: " + getPose());
                            }
                            // Not at last path, keep going
                            breakFollowing();
                            pathStartTimes[chainIndex] = System.currentTimeMillis();
                            isBusy = true;
                            followingPathChain = true;
                            chainIndex++;
                            currentPath = currentPathChain.getPath(chainIndex);
                            closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
                        } else {
                            // At last path, run some end detection stuff
                            // set isBusy to false if at end
                            if (!reachedParametricPathEnd) {
                                reachedParametricPathEnd = true;
                                reachedParametricPathEndTime = System.currentTimeMillis();
                            }

                            if ((System.currentTimeMillis() - reachedParametricPathEndTime > currentPath.getPathEndTimeoutConstraint()) ||
                                    (poseUpdater.getVelocity().getMagnitude() < currentPath.getPathEndVelocityConstraint()
                                            && MathFunctions.distance(poseUpdater.getPose(), closestPose) < currentPath.getPathEndTranslationalConstraint() &&
                                            MathFunctions.getSmallestAngleDifference(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()) < currentPath.getPathEndHeadingConstraint())) {
                                if (holdPositionAtEnd) {
                                    holdPositionAtEnd = false;
                                    holdPoint(new BezierPoint(currentPath.getLastControlPoint()), currentPath.getHeadingGoal(1));
                                } else {
                                    if (logDebug && isBusy) {
                                        Log.d("Follower_final_logger::", "isAtParametricEnd:" + currentPath.isAtParametricEnd()
                                                + " | isBusy: " + isBusy
                                                + " | closestPose:" + closestPose
                                                + " | Pose: " + getPose()
                                                + " | t-value: " + String.format("%3.5f", currentPath.getClosestPointTValue())
                                                + " | velocity: " + String.format("%3.2f", poseUpdater.getVelocity().getMagnitude())
                                                + " | distance: " + String.format("%3.2f", MathFunctions.distance(poseUpdater.getPose(), closestPose))
                                                + " | heading (degree): " + String.format("%3.2f", Math.toDegrees(MathFunctions.getSmallestAngleDifference(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal())))
                                        );
                                    }

                                    breakFollowing();
                                }
                            }
                        }
                    }
                }
            }
        } else {
            velocities.add(poseUpdater.getVelocity());
            velocities.remove(velocities.get(velocities.size() - 1));

            calculateAveragedVelocityAndAcceleration();

            drivePowers = driveVectorScaler.getDrivePowers(getCentripetalForceCorrection(), teleopHeadingVector, teleopDriveVector, poseUpdater.getPose().getHeading());

            for (int i = 0; i < motors.size(); i++) {
                if (Math.abs(motors.get(i).getPower() - drivePowers[i]) > FollowerConstants.motorCachingThreshold) {
                    double voltageNormalized = getVoltageNormalized();

                    if (useVoltageCompensationInTeleOp) {
                        motors.get(i).setPower(drivePowers[i] * voltageNormalized);
                    } else {
                        motors.get(i).setPower(drivePowers[i]);
                    }
                }
            }
        }
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
            teleopDriveVector.rotateVector(getPose().getHeading());
        }

        teleopHeadingVector.setComponents(teleopDriveValues[2], getPose().getHeading());
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

    /**
     * This checks if any PathCallbacks should be run right now, and runs them if applicable.
     */
    public void updateCallbacks() {
        for (PathCallback callback : currentPathChain.getCallbacks()) {
            if (!callback.hasBeenRun()) {
                if (callback.getType() == PathCallback.PARAMETRIC) {
                    // parametric call back
                    if (chainIndex == callback.getIndex() && (getCurrentTValue() >= callback.getStartCondition() || MathFunctions.roughlyEquals(getCurrentTValue(), callback.getStartCondition()))) {
                        callback.run();
                    }
                } else {
                    // time based call back
                    if (chainIndex >= callback.getIndex() && System.currentTimeMillis() - pathStartTimes[callback.getIndex()] > callback.getStartCondition()) {
                        callback.run();
                    }

                }
            }
        }
    }

    /**
     * This resets the PIDFs and stops following the current Path.
     */
    public void breakFollowing() {
        teleopDrive = false;
        setMotorsToFloat();
        holdingPosition = false;
        isBusy = false;
        reachedParametricPathEnd = false;
        secondaryDrivePIDF.reset();
        drivePIDF.reset();
        secondaryHeadingPIDF.reset();
        headingPIDF.reset();
        secondaryTranslationalPIDF.reset();
        secondaryTranslationalIntegral.reset();
        secondaryTranslationalIntegralVector = new Vector();
        previousSecondaryTranslationalIntegral = 0;
        translationalPIDF.reset();
        translationalIntegral.reset();
        translationalIntegralVector = new Vector();
        previousTranslationalIntegral = 0;
        driveVector = new Vector();
        headingVector = new Vector();
        translationalVector = new Vector();
        centripetalVector = new Vector();
        correctiveVector = new Vector();
        driveError = 0;
        headingError = 0;
        rawDriveError = 0;
        previousRawDriveError = 0;
        driveErrors = new double[2];
        for (int i = 0; i < driveErrors.length; i++) {
            driveErrors[i] = 0;
        }
        driveKalmanFilter.reset();

        for (int i = 0; i < AVERAGED_VELOCITY_SAMPLE_NUMBER; i++) {
            velocities.add(new Vector());
        }
        for (int i = 0; i < AVERAGED_VELOCITY_SAMPLE_NUMBER / 2; i++) {
            accelerations.add(new Vector());
        }
        calculateAveragedVelocityAndAcceleration();
        teleopDriveValues = new double[3];
        teleopDriveVector = new Vector();
        teleopHeadingVector = new Vector();

        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setPower(0);
        }

        zeroVelocityDetectedTimer = null;
    }

    /**
     * This returns if the Follower is currently following a Path or a PathChain.
     *
     * @return returns if the Follower is busy.
     */
    public boolean isBusy() {
        return isBusy;
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
            return new Vector(driveVectorScaler.getMaxPowerScaling(), currentPath.getClosestPointTangentVector().getTheta());
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
                    return new Vector(driveVectorScaler.getMaxPowerScaling(), currentPath.getClosestPointTangentVector().getTheta());
                }
            } else {
                distanceToGoal = currentPath.length() * (1 - currentPath.getClosestPointTValue());
            }
        } else {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(getPose().getX() - currentPath.getLastControlPoint().getX(), getPose().getY() - currentPath.getLastControlPoint().getY());
            distanceToGoal = MathFunctions.dotProduct(currentPath.getEndTangent(), offset);
        }

        driveError = getDriveVelocityError(distanceToGoal);

        if (Math.abs(driveError) < drivePIDFSwitch && useSecondaryDrivePID) {
            secondaryDrivePIDF.updateError(driveError);
            driveVector = new Vector(MathFunctions.clamp(secondaryDrivePIDF.runPIDF() + secondaryDrivePIDFFeedForward * MathFunctions.getSign(driveError), -driveVectorScaler.getMaxPowerScaling(), driveVectorScaler.getMaxPowerScaling()), currentPath.getClosestPointTangentVector().getTheta());
            return MathFunctions.copyVector(driveVector);
        }

        drivePIDF.updateError(driveError);
        driveVector = new Vector(MathFunctions.clamp(drivePIDF.runPIDF() + drivePIDFFeedForward * MathFunctions.getSign(driveError), -driveVectorScaler.getMaxPowerScaling(), driveVectorScaler.getMaxPowerScaling()), currentPath.getClosestPointTangentVector().getTheta());
        return MathFunctions.copyVector(driveVector);
    }

    /**
     * This returns the velocity the robot needs to be at to make it to the end of the Path
     * at some specified deceleration (well technically just some negative acceleration).
     *
     * @return returns the projected velocity.
     */
    public double getDriveVelocityError(double distanceToGoal) {
        Vector distanceToGoalVector = MathFunctions.scalarMultiplyVector(MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector()), distanceToGoal);
        Vector velocity = new Vector(MathFunctions.dotProduct(getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta());

        Vector forwardHeadingVector = new Vector(1.0, poseUpdater.getPose().getHeading());

        double forwardVelocity = MathFunctions.dotProduct(forwardHeadingVector, velocity);
        double forwardDistanceToGoal = MathFunctions.dotProduct(forwardHeadingVector, distanceToGoalVector);
        double forwardVelocityGoal = MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() * forwardZeroPowerAcceleration * (forwardDistanceToGoal <= 0 ? 1 : -1) * forwardDistanceToGoal));
        double forwardVelocityZeroPowerDecay = forwardVelocity - MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(forwardVelocity, 2) + 2 * forwardZeroPowerAcceleration * Math.abs(forwardDistanceToGoal)));

        Vector lateralHeadingVector = new Vector(1.0, poseUpdater.getPose().getHeading() - Math.PI / 2);
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
        headingError = MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()) * MathFunctions.getSmallestAngleDifference(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal());
        if (Math.abs(headingError) < headingPIDFSwitch && useSecondaryHeadingPID) {
            secondaryHeadingPIDF.updateError(headingError);
            headingVector = new Vector(MathFunctions.clamp(secondaryHeadingPIDF.runPIDF() + secondaryHeadingPIDFFeedForward * MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()), -driveVectorScaler.getMaxPowerScaling(), driveVectorScaler.getMaxPowerScaling()), poseUpdater.getPose().getHeading());
            return MathFunctions.copyVector(headingVector);
        }
        headingPIDF.updateError(headingError);
        headingVector = new Vector(MathFunctions.clamp(headingPIDF.runPIDF() + headingPIDFFeedForward * MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()), -driveVectorScaler.getMaxPowerScaling(), driveVectorScaler.getMaxPowerScaling()), poseUpdater.getPose().getHeading());
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

        if (corrective.getMagnitude() > driveVectorScaler.getMaxPowerScaling()) {
            return MathFunctions.addVectors(centripetal, MathFunctions.scalarMultiplyVector(translational, driveVectorScaler.findNormalizingScaling(centripetal, translational)));
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

        translationalVector.setMagnitude(MathFunctions.clamp(translationalVector.getMagnitude(), 0, driveVectorScaler.getMaxPowerScaling()));

        this.translationalVector = MathFunctions.copyVector(translationalVector);

        return translationalVector;
    }

    /**
     * This returns the raw translational error, or how far off the closest point the robot is.
     *
     * @return This returns the raw translational error as a Vector.
     */
    public Vector getTranslationalError() {
        Vector error = new Vector();
        double x = closestPose.getX() - poseUpdater.getPose().getX();
        double y = closestPose.getY() - poseUpdater.getPose().getY();
        error.setOrthogonalComponents(x, y);
        return error;
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
        centripetalVector = new Vector(MathFunctions.clamp(centripetalScaling * FollowerConstants.mass * Math.pow(MathFunctions.dotProduct(poseUpdater.getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), 2) * curvature, -driveVectorScaler.getMaxPowerScaling(), driveVectorScaler.getMaxPowerScaling()), currentPath.getClosestPointTangentVector().getTheta() + Math.PI / 2 * MathFunctions.getSign(currentPath.getClosestPointNormalVector().getTheta()));
        return centripetalVector;
    }

    /**
     * This returns the closest pose to the robot on the Path the Follower is currently following.
     * This closest pose is calculated through a binary search method with some specified number of
     * steps to search. By default, 10 steps are used, which should be more than enough.
     *
     * @return returns the closest pose.
     */
    public Pose getClosestPose() {
        return closestPose;
    }

    /**
     * This returns whether the follower is at the parametric end of its current Path.
     * The parametric end is determined by if the closest Point t-value is greater than some specified
     * end t-value.
     * If running a PathChain, this returns true only if at parametric end of last Path in the PathChain.
     *
     * @return returns whether the Follower is at the parametric end of its Path.
     */
    public boolean atParametricEnd() {
        if (followingPathChain) {
            if (chainIndex == currentPathChain.size() - 1) return currentPath.isAtParametricEnd();
            return false;
        }
        return currentPath.isAtParametricEnd();
    }

    /**
     * This returns the t value of the closest point on the current Path to the robot
     * In the absence of a current Path, it returns 1.0.
     *
     * @return returns the current t value.
     */
    public double getCurrentTValue() {
        if (isBusy) return currentPath.getClosestPointTValue();
        return 1.0;
    }

    /**
     * This returns the current path number. For following Paths, this will return 0. For PathChains,
     * this will return the current path number. For holding Points, this will also return 0.
     *
     * @return returns the current path number.
     */
    public double getCurrentPathNumber() {
        if (!followingPathChain) return 0;
        return chainIndex;
    }

    /**
     * This returns a new PathBuilder object for easily building PathChains.
     *
     * @return returns a new PathBuilder object.
     */
    public PathBuilder pathBuilder() {
        return new PathBuilder();
    }

    /**
     * This writes out information about the various motion Vectors to the Telemetry specified.
     *
     * @param telemetry this is an instance of Telemetry or the FTC Dashboard telemetry that this
     *                  method will use to output the debug data.
     */
    public void telemetryDebug(MultipleTelemetry telemetry) {
        telemetry.addData("follower busy", isBusy());
        telemetry.addData("heading error", headingError);
        telemetry.addData("heading vector magnitude", headingVector.getMagnitude());
        telemetry.addData("corrective vector magnitude", correctiveVector.getMagnitude());
        telemetry.addData("corrective vector heading", correctiveVector.getTheta());
        telemetry.addData("translational error magnitude", getTranslationalError().getMagnitude());
        telemetry.addData("translational error direction", getTranslationalError().getTheta());
        telemetry.addData("translational vector magnitude", translationalVector.getMagnitude());
        telemetry.addData("translational vector heading", translationalVector.getMagnitude());
        telemetry.addData("centripetal vector magnitude", centripetalVector.getMagnitude());
        telemetry.addData("centripetal vector heading", centripetalVector.getTheta());
        telemetry.addData("drive error", driveError);
        telemetry.addData("drive vector magnitude", driveVector.getMagnitude());
        telemetry.addData("drive vector heading", driveVector.getTheta());
        telemetry.addData("x", getPose().getX());
        telemetry.addData("y", getPose().getY());
        telemetry.addData("heading", getPose().getHeading());
        telemetry.addData("total heading", poseUpdater.getTotalHeading());
        telemetry.addData("velocity magnitude", getVelocity().getMagnitude());
        telemetry.addData("velocity heading", getVelocity().getTheta());
        driveKalmanFilter.debug(telemetry);
        telemetry.update();
        if (drawOnDashboard) {
            Drawing.drawDebug(this);
        }
    }

    /**
     * This writes out information about the various motion Vectors to the Telemetry specified.
     *
     * @param telemetry this is an instance of Telemetry or the FTC Dashboard telemetry that this
     *                  method will use to output the debug data.
     */
    public void telemetryDebug(Telemetry telemetry) {
        telemetryDebug(new MultipleTelemetry(telemetry));
    }

    /**
     * This returns the total number of radians the robot has turned.
     *
     * @return the total heading.
     */
    public double getTotalHeading() {
        return poseUpdater.getTotalHeading();
    }

    /**
     * This returns the current Path the Follower is following. This can be null.
     *
     * @return returns the current Path.
     */
    public Path getCurrentPath() {
        return currentPath;
    }

    /**
     * This returns the pose tracker for the robot to draw on the Dashboard.
     *
     * @return returns the pose tracker
     */
    public DashboardPoseTracker getDashboardPoseTracker() {
        return dashboardPoseTracker;
    }

    /**
     * This resets the IMU, if applicable.
     */
    private void resetIMU() throws InterruptedException {
        poseUpdater.resetIMU();
    }

    private void debugLog() {
        Log.d("Follower_logger::", "isAtParametricEnd:" + currentPath.isAtParametricEnd()
                + " | isBusy: " + isBusy
                + " | closestPose:" + closestPose
                + " | Pose: " + getPose()
                + " | t-value: " + String.format("%3.5f",currentPath.getClosestPointTValue())
                + " | zeroVelocityTimer: " +  String.format("%3.2f",(zeroVelocityDetectedTimer==null?0.0: zeroVelocityDetectedTimer.milliseconds()))
                + " | velocity: " + String.format("%3.2f",poseUpdater.getVelocity().getMagnitude())
                + " | distance: " +  String.format("%3.2f",MathFunctions.distance(poseUpdater.getPose(), closestPose))
                + " | heading (degree): " +  String.format("%3.2f",Math.toDegrees(MathFunctions.getSmallestAngleDifference(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal())))
        );
    }

    //Thanks to team 21229 Quality Control for creating this algorithm to detect if the robot is stuck.
    /**
     * @return true if the robot is stuck and false otherwise
     */
    public boolean isRobotStuck() {
        return zeroVelocityDetectedTimer != null;
    }

    /**
     * Draws everything in the debug() method on the dashboard
     */

    public void drawOnDashBoard() {
        if (drawOnDashboard) {
            Drawing.drawDebug(this);
        }
    }

    public boolean isLocalizationNAN() {
        return poseUpdater.getLocalizer().isNAN();
    }

    /**
     * @return The last cached voltage measurement.
     */
    public double getVoltage() {
        if (voltageTimer.seconds() > cacheInvalidateSeconds && cacheInvalidateSeconds >= 0) {
            cached = false;
        }

        if (!cached)
            refreshVoltage();

        return voltage;
    }

    /**
     * @return A scalar that normalizes power outputs to the nominal voltage from the current voltage.
     */
    public double getVoltageNormalized() {
        return Math.min(nominalVoltage / getVoltage(), 1);
    }

    /**
     * Overrides the voltage cooldown.
     */
    public void refreshVoltage() {
        cached = true;
        voltage = voltageSensor.getVoltage();
        voltageTimer.reset();
    }

    /** Turns a certain amount of degrees left
     * @param radians the amount of radians to turn
     * @param isLeft true if turning left, false if turning right
     */
    public void turn(double radians, boolean isLeft) {
        Pose temp = new Pose(getPose().getX(), getPose().getY(), getPose().getHeading() + (isLeft ? radians : -radians));
        holdPoint(temp);
        isTurning = true;
        isBusy = true;
    }

    /** Turns to a specific heading
     * @param radians the heading in radians to turn to
     */
    public void turnTo(double radians) {
        holdPoint(new Pose(getPose().getX(), getPose().getY(), Math.toRadians(radians)));
        isTurning = true;
        isBusy = true;
    }

    /** Turns to a specific heading in degrees
     * @param degrees the heading in degrees to turn to
     */
    public void turnToDegrees(double degrees) {
        turnTo(Math.toRadians(degrees));
    }

    /** Turns a certain amount of degrees left
     * @param degrees the amount of degrees to turn
     * @param isLeft true if turning left, false if turning right
     */
    public void turnDegrees(double degrees, boolean isLeft) {
        turn(Math.toRadians(degrees), isLeft);
    }

    public boolean isTurning() {
        return isTurning;
    }

    /**
     * This will update the PIDF coefficients for primary Heading PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setHeadingPIDF(CustomPIDFCoefficients set){
        headingPIDF.setCoefficients(set);
    }

    /**
     * This will update the PIDF coefficients for primary Translational PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setTranslationalPIDF(CustomPIDFCoefficients set){
        translationalPIDF.setCoefficients(set);
    }

    /**
     * This will update the PIDF coefficients for primary Drive PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setDrivePIDF(CustomFilteredPIDFCoefficients set){
        drivePIDF.setCoefficients(set);
    }

    /**
     * This will update the PIDF coefficients for secondary Heading PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setSecondaryHeadingPIDF(CustomPIDFCoefficients set){
        secondaryHeadingPIDF.setCoefficients(set);
    }

    /**
     * This will update the PIDF coefficients for secondary Translational PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setSecondaryTranslationalPIDF(CustomPIDFCoefficients set){
        secondaryTranslationalPIDF.setCoefficients(set);
    }

    /**
     * This will update the PIDF coefficients for secondary Drive PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setSecondaryDrivePIDF(CustomFilteredPIDFCoefficients set){
        secondaryDrivePIDF.setCoefficients(set);
    }


    /**
     * Checks if the robot is at a certain pose within certain tolerances
     * @param pose Pose to compare with the current pose
     * @param xTolerance Tolerance for the x position
     * @param yTolerance Tolerance for the y position
     * @param headingTolerance Tolerance for the heading
     */
    public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
        return Math.abs(pose.getX() - getPose().getX()) < xTolerance && Math.abs(pose.getY() - getPose().getY()) < yTolerance && Math.abs(pose.getHeading() - getPose().getHeading()) < headingTolerance;
    }

    /**
     * Checks if the robot is at a certain pose within certain tolerances
     * @param pose Pose to compare with the current pose
     * @param xTolerance Tolerance for the x position
     * @param yTolerance Tolerance for the y position
     */
    public boolean atPose(Pose pose, double xTolerance, double yTolerance) {
        return Math.abs(pose.getX() - getPose().getX()) < xTolerance && Math.abs(pose.getY() - getPose().getY()) < yTolerance;
    }

    public double getHeadingError() {
        return headingError;
    }
}
