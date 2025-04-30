package com.pedropathing.follower;

import static com.pedropathing.follower.old.FollowerConstants.automaticHoldEnd;
import static com.pedropathing.follower.old.FollowerConstants.cacheInvalidateSeconds;
import static com.pedropathing.follower.old.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.old.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.old.FollowerConstants.nominalVoltage;
import static com.pedropathing.follower.old.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.old.FollowerConstants.rightRearMotorName;
import static com.pedropathing.follower.old.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.old.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.old.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.old.FollowerConstants.rightRearMotorDirection;
import static com.pedropathing.follower.old.FollowerConstants.useVoltageCompensationInAuto;
import static com.pedropathing.follower.old.FollowerConstants.useVoltageCompensationInTeleOp;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.old.FollowerConstants;
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
import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private Drivetrain drivetrain;
    private ErrorHandler errorHandler;

    public PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    private Pose closestPose;

    private Path currentPath;

    private PathChain currentPathChain;

    private int BEZIER_CURVE_SEARCH_LIMIT;


    private int chainIndex;

    private long[] pathStartTimes;

    private boolean followingPathChain;
    private boolean holdingPosition;
    private boolean isBusy, isTurning;
    private boolean reachedParametricPathEnd;
    private boolean holdPositionAtEnd;
    private boolean teleopDrive;

    private double globalMaxPower = 1;
    private double holdPointTranslationalScaling;
    private double holdPointHeadingScaling;

    private long reachedParametricPathEndTime;

    private double[] drivePowers;

    private double centripetalScaling;

    private PIDFController secondaryTranslationalPIDF;
    private PIDFController secondaryTranslationalIntegral;
    private PIDFController translationalPIDF;
    private PIDFController translationalIntegral;
    private PIDFController secondaryHeadingPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController secondaryDrivePIDF;
    private FilteredPIDFController drivePIDF;

    private double turnHeadingErrorThreshold;

    private double maxPowerScaling = 1.0;

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
        maxPowerScaling = FollowerConstants.maxPower;
        BEZIER_CURVE_SEARCH_LIMIT = FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT;
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
       //TODO add switch
        drivetrain = new Mecanum(FollowerConstants.frontLeftVector, maxPowerScaling);
        errorHandler = new ErrorHandler(poseUpdater, drivetrain);


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
        //TODO add switch
        drivetrain = new Mecanum(FollowerConstants.frontLeftVector, maxPowerScaling);
        errorHandler = new ErrorHandler(poseUpdater, drivetrain);

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
        maxPowerScaling = set;
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
     * This sets the current pose in the PoseUpdater without using offsets.
     *
     * @param pose The pose to set the current pose to.
     */
    public void setPose(Pose pose) {
        poseUpdater.setPose(pose);
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
        maxPowerScaling = globalMaxPower;
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
        maxPowerScaling = maxPower;
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
     * Resumes pathing
     */
    public void resumePathFollowing() {
        pathStartTimes = new long[currentPathChain.size()];
        pathStartTimes[0] = System.currentTimeMillis();
        isBusy = true;
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
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
     * Calls an update to the ErrorHandler, which updates the robot's current errors and corrective vectors.
     */
    public void updateErrors() {
        errorHandler.update(this);
    }

    /**
     * This calls an update to the PoseUpdater, which updates the robot's current position estimate.
     * This also updates all the Follower's PIDFs, which updates the motor powers.
     */
    public void update() {
        updatePose();
        updateErrors();

        if (!teleopDrive) {
            if (currentPath != null) {
                if (holdingPosition) {
                    closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), 1);

                    drivePowers = drivetrain.getDrivePowers(MathFunctions.scalarMultiplyVector(getTranslationalCorrection(), holdPointTranslationalScaling), MathFunctions.scalarMultiplyVector(getHeadingVector(), holdPointHeadingScaling), new Vector(), poseUpdater.getPose().getHeading());

                    updateErrors();

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

                    if(getHeadingError() < turnHeadingErrorThreshold && isTurning) {
                        isTurning = false;
                        isBusy = false;
                    }
                } else {

                    if (isBusy) {
                        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_SEARCH_LIMIT);

                        if (followingPathChain) updateCallbacks();

                        updateErrors();

                        drivePowers = drivetrain.getDrivePowers(getCorrectiveVector(), getHeadingVector(), getDriveVector(), poseUpdater.getPose().getHeading());

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

                            updateErrors();
                        } else {
                            // At last path, run some end detection stuff
                            // set isBusy to false if at end
                            if (!reachedParametricPathEnd) {
                                reachedParametricPathEnd = true;
                                reachedParametricPathEndTime = System.currentTimeMillis();
                            }
                            updateErrors();

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
            updateErrors();

            errorHandler.teleopUpdate();

            drivePowers = drivetrain.getDrivePowers(getCentripetalForceCorrection(), getTeleopHeadingVector(), getTeleopDriveVector(), poseUpdater.getPose().getHeading());

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
        if (logDebug) {
            Log.d("Follower_logger", "Breaking following");
        }
        errorHandler.breakFollowing();
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
        translationalPIDF.reset();
        translationalIntegral.reset();
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
    public void multipleTelemetryDebug(MultipleTelemetry telemetry) {
        telemetry.addData("follower busy", isBusy());
        telemetry.addData("heading error", getHeadingError());
        telemetry.addData("heading vector magnitude", getHeadingVector().getMagnitude());
        telemetry.addData("corrective vector magnitude", getCorrectiveVector().getMagnitude());
        telemetry.addData("corrective vector heading", getCorrectiveVector().getTheta());
        telemetry.addData("translational error magnitude", getTranslationalError().getMagnitude());
        telemetry.addData("translational error direction", getTranslationalError().getTheta());
        telemetry.addData("translational vector magnitude", getTranslationalVector().getMagnitude());
        telemetry.addData("translational vector heading", getTranslationalVector().getMagnitude());
        telemetry.addData("centripetal vector magnitude", getCentripetalVector().getMagnitude());
        telemetry.addData("centripetal vector heading", getCentripetalVector().getTheta());
        telemetry.addData("drive error", getDriveError());
        telemetry.addData("drive vector magnitude", getDriveVector().getMagnitude());
        telemetry.addData("drive vector heading", getDriveVector().getTheta());
        telemetry.addData("x", getPose().getX());
        telemetry.addData("y", getPose().getY());
        telemetry.addData("heading", getPose().getHeading());
        telemetry.addData("total heading", poseUpdater.getTotalHeading());
        telemetry.addData("velocity magnitude", getVelocity().getMagnitude());
        telemetry.addData("velocity heading", getVelocity().getTheta());
        getDriveKalmanFilter().debug(telemetry);
        telemetry.update();
//        if (drawOnDashboard) {
//            Drawing.drawDebug(NewFollower);
//        } BAH
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
//        if (drawOnDashboard) {
//            Drawing.drawDebug(this);
//        } BAH
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

    /**
     * Sets the maximum power that can be used by the drive vector scaler. Clamped between 0 and 1.
     *
     * @param maxPowerScaling setting the max power scaling
     */
    public void setMaxPowerScaling(double maxPowerScaling) {
        this.maxPowerScaling = MathFunctions.clamp(maxPowerScaling, 0, 1);
    }

    /**
     * Gets the maximum power that can be used by the drive vector scaler. Ranges between 0 and 1.
     *
     * @return returns the max power scaling
     */
    public double getMaxPowerScaling() {
        return maxPowerScaling;
    }

    /**
     * Returns the useDrive boolean
     */
    public boolean getUseDrive() {
        return useDrive;
    }

    /**
     * Returns the useHeading boolean
     */
    public boolean getUseHeading() {
        return useHeading;
    }

    /**
     * Returns the useTranslational boolean
     */
    public boolean getUseTranslational() {
        return useTranslational;
    }

    /**
     * Returns the useCentripetal boolean
     */
    public boolean getUseCentripetal() {
        return useCentripetal;
    }

    /**
     * Return the teleopDrive boolean
     */
    public boolean getTeleopDrive() {
        return teleopDrive;
    }

    /**
     * Returns the chainIndex of the current PathChain
     */
    public int getChainIndex() {
        return chainIndex;
    }

    /**
     * Returns the current PathChain
     */
    public PathChain getCurrentPathChain() {
        return currentPathChain;
    }

    /**
     * Returns if following a path chain
     */
    public boolean getFollowingPathChain() {
        return followingPathChain;
    }

    /**
     * Return the centripetal scaling
     */
    public double getCentripetalScaling() {
        return centripetalScaling;
    }

    public PIDFController getSecondaryTranslationalPIDF() {
        return secondaryTranslationalPIDF;
    }

    public PIDFController getSecondaryTranslationalIntegral() {
        return secondaryTranslationalIntegral;
    }

    public PIDFController getTranslationalPIDF() {
        return translationalPIDF;
    }

    public PIDFController getTranslationalIntegral() {
        return translationalIntegral;
    }

    public PIDFController getSecondaryHeadingPIDF() {
        return secondaryHeadingPIDF;
    }

    public PIDFController getHeadingPIDF() {
        return headingPIDF;
    }

    public FilteredPIDFController getSecondaryDrivePIDF() {
        return secondaryDrivePIDF;
    }

    public FilteredPIDFController getDrivePIDF() {
        return drivePIDF;
    }

    public boolean isTeleopDrive() {
        return teleopDrive;
    }

    public Vector getCentripetalVector() {
        return errorHandler.getCentripetalVector();
    }

    public Vector getTranslationalVector() {
        return errorHandler.getTranslationalVector();
    }

    public Vector getTeleopHeadingVector() {
        return errorHandler.getTeleopHeadingVector();
    }

    public Vector getTeleopDriveVector() {
        return errorHandler.getTeleopDriveVector();
    }

    public Vector getTranslationalIntegralVector() {
        return errorHandler.getTranslationalIntegralVector();
    }

    public Vector getAverageAcceleration() {
        return errorHandler.getAverageAcceleration();
    }

    public Vector getSecondaryTranslationalIntegralVector() {
        return errorHandler.getSecondaryTranslationalIntegralVector();
    }

    public double getHeadingError() {
        return errorHandler.getHeadingError();
    }

    public double getDriveError() {
        return errorHandler.getDriveError();
    }

    public double getRawDriveError() {
        return errorHandler.getRawDriveError();
    }

    public double[] getDriveErrors() {
        return errorHandler.getDriveErrors();
    }

    public Vector getDriveVector() {
        return errorHandler.getDriveVector();
    }

    public Vector getCorrectiveVector() {
        return errorHandler.getCorrectiveVector();
    }

    public Vector getHeadingVector() {
        return errorHandler.getHeadingVector();
    }

    public Vector getTranslationalError() {
        return errorHandler.getTranslationalError();
    }

    public Vector getTranslationalCorrection() {
        return errorHandler.getTranslationalCorrection();
    }

    public Vector getCentripetalForceCorrection() {
        return errorHandler.getCentripetalForceCorrection();
    }

    public KalmanFilter getDriveKalmanFilter() {
        return errorHandler.getDriveKalmanFilter();
    }


}
