package com.pedropathing.localization.localizers;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import math.MathFunctions;
import math.Vector;
import static com.pedropathing.localization.constants.OTOSConstants.*;

/**
 * This is the OTOSLocalizer class. This class extends the Localizer superclass and is a
 * localizer that uses the SparkFun OTOS. The diagram below, which is modified from
 * Road Runner, shows a typical set up.
 *
 * The view is from the top of the robot looking downwards.
 *
 * left on robot is the y positive direction
 *
 * forward on robot is the x positive direction
 *
 *                         forward (x positive)
 *                                △
 *                                |
 *                                |
 *                         /--------------\
 *                         |              |
 *                         |              |
 *                         | ||        || |
 *  left (y positive) <--- | ||        || |  
 *                         |     ____     |
 *                         |     ----     |
 *                         \--------------/
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 7/20/2024
 */
public class OTOSLocalizer extends Localizer {
    private HardwareMap hardwareMap;
    private Pose startPose;
    private SparkFunOTOS otos;
    private SparkFunOTOS.Pose2D otosPose;
    private SparkFunOTOS.Pose2D otosVel;
    private SparkFunOTOS.Pose2D otosAcc;
    private double previousHeading;
    private double totalHeading;

    /**
     * This creates a new OTOSLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public OTOSLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    /**
     * This creates a new OTOSLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map the HardwareMap
     * @param setStartPose the Pose to start from
     */

    public OTOSLocalizer(HardwareMap map, Pose setStartPose) {
        hardwareMap = map;

        otos = hardwareMap.get(SparkFunOTOS.class, hardwareMapName);

        otos.setLinearUnit(linearUnit);
        otos.setAngularUnit(angleUnit);
        otos.setOffset(offset);
        otos.setLinearScalar(linearScalar);
        otos.setAngularScalar(angularScalar);

        otos.calibrateImu();
        otos.resetTracking();

        setStartPose(setStartPose);

        otosPose = new SparkFunOTOS.Pose2D();
        otosVel = new SparkFunOTOS.Pose2D();
        otosAcc = new SparkFunOTOS.Pose2D();
        totalHeading = 0;
        previousHeading = startPose.getHeading();

        resetOTOS();
    }



    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        Pose pose = new Pose(otosPose.x, otosPose.y, otosPose.h);

        Vector vec = pose.getVector();
        vec.rotateVector(startPose.getHeading());

        return MathFunctions.addPoses(startPose, new Pose(vec.getXComponent(), vec.getYComponent(), pose.getHeading()));
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return new Pose(otosVel.x, otosVel.y, otosVel.h);
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return getVelocity().getVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        resetOTOS();
        Pose setOTOSPose = MathFunctions.subtractPoses(setPose, startPose);
        otos.setPosition(new SparkFunOTOS.Pose2D(setOTOSPose.getX(), setOTOSPose.getY(), setOTOSPose.getHeading()));
    }

    /**
     * This updates the total heading of the robot. The OTOS handles all other updates itself.
     */
    @Override
    public void update() {
        otos.getPosVelAcc(otosPose,otosVel,otosAcc);
        totalHeading += MathFunctions.getSmallestAngleDifference(otosPose.h, previousHeading);
        previousHeading = otosPose.h;
    }

    /**
     * This resets the OTOS.
     */
    public void resetOTOS() {
        otos.resetTracking();
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from OTOS
     * ticks to inches. For the OTOS, this value is the same as the lateral multiplier.
     * This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double getForwardMultiplier() {
        return otos.getLinearScalar();
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * OTOS ticks to inches. For the OTOS, this value is the same as the forward multiplier.
     * This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double getLateralMultiplier() {
        return otos.getLinearScalar();
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from OTOS ticks
     * to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double getTurningMultiplier() {
        return otos.getAngularScalar();
    }

    /**
     * This does nothing since this localizer does not use the IMU.
     */
    public void resetIMU() {
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns whether the robot's position is NaN
     */
    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }
}
