package com.pedropathing.follower;

import com.pedropathing.control.Controller;
import com.pedropathing.control.Controller.Coefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;

/**
 * This is the FollowerConstants class. It holds many constants and parameters for various parts of
 * the Follower. This is here to allow for easier tuning of Pedro Pathing, as well as concentrate
 * everything tunable for the Paths themselves in one place.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 5/1/2025
 */

public class FollowerConstants {

    /**
     * Translational  coefficients
     * Default Value: new Coefficients(0.1,0,0,0);
     */
    public Controller.Coefficients coefficientsTranslational = new PIDFCoefficients(
            0.1,
            0,
            0,
            0.015);

    /**
     * Translational Integral
     * Default Value: new Coefficients(0,0,0,0);
     */
    public PIDFCoefficients integralTranslational = new PIDFCoefficients(
            0,
            0,
            0,
            0);

    /**
     * Heading error  coefficients
     * Default Value: new Coefficients(1,0,0,0);
     */
    public Coefficients coefficientsHeading = new PIDFCoefficients(
            1,
            0,
            0,
            0.01);


    /**
     * Drive  coefficients
     * Default Value: new Coefficients(0.025,0,0.00001,0.6,0);
     */
    public Coefficients coefficientsDrive = new FilteredPIDFCoefficients(
            0.025,
            0,
            0.00001,
            0.6,
            0.01);

    /**
     * Secondary translational  coefficients (don't use integral).
     * Default Value: new Coefficients(0.3, 0, 0.01, 0)
     */
    public Coefficients coefficientsSecondaryTranslational = new PIDFCoefficients(
            0.3,
            0,
            0.01,
            0);

    /**
     * Secondary translational Integral value.
     * Default Value: new Coefficients(0, 0, 0, 0)
     */
    public PIDFCoefficients integralSecondaryTranslational = new PIDFCoefficients(
            0,
            0,
            0,
            0.015);

    /**
     * The limit at which the heading  switches between the main and secondary heading s.
     * Default Value: Math.PI / 20
     */
    public double headingSwitch = Math.PI / 20;

    /**
     * Secondary heading error  coefficients.
     * Default Value: new Coefficients(5, 0, 0.08, 0)
     */
    public Coefficients coefficientsSecondaryHeading = new PIDFCoefficients(
            5,
            0,
            0.08,
            0.01);

    /**
     * The limit at which the heading  switches between the main and secondary drive s.
     * Default Value: 20
     */
    public double driveSwitch = 20;

    /**
     * Secondary drive  coefficients.
     * Default Value: new Coefficients(0.02, 0, 0.000005, 0.6, 0)
     */
    public Coefficients coefficientsSecondaryDrive = new FilteredPIDFCoefficients(
            0.02,
            0,
            0.000005,
            0.6,
            0.01);

    /**
     * This scales the translational error correction power when the Follower is holding a Point.
     * Default Value: 0.45
     */
    public double holdPointTranslationalScaling = 0.45;

    /**
     * This scales the heading error correction power when the Follower is holding a Point.
     * Default Value: 0.35
     */
    public double holdPointHeadingScaling = 0.35;

    /**
     * This is the number of steps the search for the closest point uses. More steps lead to bigger
     * accuracy. However, more steps also take more time.
     * Default Value: 10
     */
    public int BEZIER_CURVE_SEARCH_LIMIT = 10;

    /**
     * This activates/deactivates the secondary translational . It takes over at a certain translational error
     *
     * @see #translationalSwitch
     * Default Value: false
     */
    public boolean useSecondaryTranslational = false;

    /**
     * Use the secondary heading . It takes over at a certain heading error
     *
     * @see #headingSwitch
     * Default Value: false
     */
    public boolean useSecondaryHeading = false;

    /**
     * Use the secondary drive . It takes over at a certain drive error
     *
     * @see #driveSwitch
     * Default Value: false
     */
    public boolean useSecondaryDrive = false;

    /**
     * The limit at which the translational  switches between the main and secondary translational s,
     * if the secondary PID is active.
     * Default Value: 3
     */
    public double translationalSwitch = 3;

    /**
     * Threshold that the turn and turnTo methods will be considered to be finished
     * In Radians
     * Default Value: 0.01
     */
    public double turnHeadingErrorThreshold = 0.01;

    /**
     * Centripetal force to power scaling
     * Default Value: 0.0005
     */
    public double centripetalScaling = 0.0005;

    /**
     * This is the default value for the automatic hold end. If this is set to true, the Follower will
     * automatically hold the end when it reaches the end of the Path.
     * Default Value: true
     */
    public boolean automaticHoldEnd = true;

    /**
     * This is the mass of the robot. This is used to calculate the centripetal force.
     * Default Value: 10.65
     */
    public double mass = 10.65;

    /** Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
     * if not negative, then the robot thinks that its going to go faster under 0 power
     *  Default Value: -34.62719
     * This value is found via 'ForwardZeroPowerAccelerationTuner'*/
    public double forwardZeroPowerAcceleration = -34.62719;

    /** Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
     * if not negative, then the robot thinks that its going to go faster under 0 power
     *  Default Value: -78.15554
     * This value is found via 'LateralZeroPowerAccelerationTuner'*/
    public double lateralZeroPowerAcceleration = -78.15554;

    public FollowerConstants() {
        defaults();
    }

    public FollowerConstants translationalCoefficients(Coefficients translationalCoefficients) {
        this.coefficientsTranslational = translationalCoefficients;
        return this;
    }

    public FollowerConstants headingCoefficients(Coefficients headingCoefficients) {
        this.coefficientsHeading = headingCoefficients;
        return this;
    }

    public FollowerConstants driveCoefficients(Coefficients driveCoefficients) {
        this.coefficientsDrive = driveCoefficients;
        return this;
    }

    public FollowerConstants secondaryTranslationalCoefficients(Coefficients secondaryTranslationalCoefficients) {
        this.coefficientsSecondaryTranslational = secondaryTranslationalCoefficients;
        useSecondaryTranslational = true;
        return this;
    }

    public FollowerConstants headingSwitch(double headingSwitch) {
        this.headingSwitch = headingSwitch;
        return this;
    }

    public FollowerConstants secondaryHeadingCoefficients(Coefficients secondaryHeadingCoefficients) {
        this.coefficientsSecondaryHeading = secondaryHeadingCoefficients;
        useSecondaryHeading = true;
        return this;
    }

    public FollowerConstants driveSwitch(double driveSwitch) {
        this.driveSwitch = driveSwitch;
        return this;
    }

    public FollowerConstants secondaryDriveCoefficients(Coefficients secondaryDriveCoefficients) {
        this.coefficientsSecondaryDrive = secondaryDriveCoefficients;
        useSecondaryDrive = true;
        return this;
    }

    public FollowerConstants holdPointTranslationalScaling(double holdPointTranslationalScaling) {
        this.holdPointTranslationalScaling = holdPointTranslationalScaling;
        return this;
    }

    public FollowerConstants holdPointHeadingScaling(double holdPointHeadingScaling) {
        this.holdPointHeadingScaling = holdPointHeadingScaling;
        return this;
    }

    public FollowerConstants BEZIER_CURVE_SEARCH_LIMIT(int BEZIER_CURVE_SEARCH_LIMIT) {
        this.BEZIER_CURVE_SEARCH_LIMIT = BEZIER_CURVE_SEARCH_LIMIT;
        return this;
    }

    public FollowerConstants useSecondaryTranslational(boolean useSecondaryTranslational) {
        this.useSecondaryTranslational = useSecondaryTranslational;
        return this;
    }

    public FollowerConstants useSecondaryHeading(boolean useSecondaryHeading) {
        this.useSecondaryHeading = useSecondaryHeading;
        return this;
    }

    public FollowerConstants useSecondaryDrive(boolean useSecondaryDrive) {
        this.useSecondaryDrive = useSecondaryDrive;
        return this;
    }

    public FollowerConstants translationalSwitch(double translationalSwitch) {
        this.translationalSwitch = translationalSwitch;
        return this;
    }

    public FollowerConstants turnHeadingErrorThreshold(double turnHeadingErrorThreshold) {
        this.turnHeadingErrorThreshold = turnHeadingErrorThreshold;
        return this;
    }

    public FollowerConstants centripetalScaling(double centripetalScaling) {
        this.centripetalScaling = centripetalScaling;
        return this;
    }

    public FollowerConstants automaticHoldEnd(boolean automaticHoldEnd) {
        this.automaticHoldEnd = automaticHoldEnd;
        return this;
    }

    public FollowerConstants mass(double mass) {
        this.mass = mass;
        return this;
    }

    public FollowerConstants forwardZeroPowerAcceleration(double forwardZeroPowerAcceleration) {
        this.forwardZeroPowerAcceleration = forwardZeroPowerAcceleration;
        return this;
    }

    public FollowerConstants lateralZeroPowerAcceleration(double lateralZeroPowerAcceleration) {
        this.lateralZeroPowerAcceleration = lateralZeroPowerAcceleration;
        return this;
    }

    public Coefficients getCoefficientsTranslational() {
        return coefficientsTranslational;
    }

    public void setCoefficientsTranslational(Coefficients coefficientsTranslational) {
        this.coefficientsTranslational = coefficientsTranslational;
    }

    public Coefficients getIntegralTranslational() {
        return integralTranslational;
    }

    public void setIntegralTranslational(PIDFCoefficients integralTranslational) {
        this.integralTranslational = integralTranslational;
    }

    public Coefficients getCoefficientsHeading() {
        return coefficientsHeading;
    }

    public void setCoefficientsHeading(Coefficients coefficientsHeading) {
        this.coefficientsHeading = coefficientsHeading;
    }
    public Coefficients getCoefficientsDrive() {
        return coefficientsDrive;
    }

    public void setCoefficientsDrive(Coefficients coefficientsDrive) {
        this.coefficientsDrive = coefficientsDrive;
    }

    public Coefficients getCoefficientsSecondaryTranslational() {
        return coefficientsSecondaryTranslational;
    }

    public void setCoefficientsSecondaryTranslational(Coefficients coefficientsSecondaryTranslational) {
        this.coefficientsSecondaryTranslational = coefficientsSecondaryTranslational;
        useSecondaryTranslational = true;
    }

    public PIDFCoefficients getIntegralSecondaryTranslational() {
        return integralSecondaryTranslational;
    }

    public void setIntegralSecondaryTranslational(PIDFCoefficients integralSecondaryTranslational) {
        this.integralSecondaryTranslational = integralSecondaryTranslational;
    }

    public FollowerConstants translationalIntegral(double translationalIntegral) {
        this.integralTranslational = new PIDFCoefficients(0, translationalIntegral, 0, 0);
        return this;
    }

    public double getHeadingSwitch() {
        return headingSwitch;
    }

    public void setHeadingSwitch(double headingSwitch) {
        this.headingSwitch = headingSwitch;
    }

    public Coefficients getCoefficientsSecondaryHeading() {
        return coefficientsSecondaryHeading;
    }

    public void setCoefficientsSecondaryHeading(Coefficients coefficientsSecondaryHeading) {
        this.coefficientsSecondaryHeading = coefficientsSecondaryHeading;
        useSecondaryHeading = true;
    }

    public double getDriveSwitch() {
        return driveSwitch;
    }

    public void setDriveSwitch(double driveSwitch) {
        this.driveSwitch = driveSwitch;
    }

    public Coefficients getCoefficientsSecondaryDrive() {
        return coefficientsSecondaryDrive;
    }

    public void setCoefficientsSecondaryDrive(Coefficients coefficientsSecondaryDrive) {
        this.coefficientsSecondaryDrive = coefficientsSecondaryDrive;
        useSecondaryDrive = true;
    }

    public double getHoldPointTranslationalScaling() {
        return holdPointTranslationalScaling;
    }

    public void setHoldPointTranslationalScaling(double holdPointTranslationalScaling) {
        this.holdPointTranslationalScaling = holdPointTranslationalScaling;
    }

    public double getHoldPointHeadingScaling() {
        return holdPointHeadingScaling;
    }

    public void setHoldPointHeadingScaling(double holdPointHeadingScaling) {
        this.holdPointHeadingScaling = holdPointHeadingScaling;
    }

    public int getBEZIER_CURVE_SEARCH_LIMIT() {
        return BEZIER_CURVE_SEARCH_LIMIT;
    }

    public void setBEZIER_CURVE_SEARCH_LIMIT(int BEZIER_CURVE_SEARCH_LIMIT) {
        this.BEZIER_CURVE_SEARCH_LIMIT = BEZIER_CURVE_SEARCH_LIMIT;
    }

    public boolean isUseSecondaryTranslational() {
        return useSecondaryTranslational;
    }

    public void setUseSecondaryTranslational(boolean useSecondaryTranslational) {
        this.useSecondaryTranslational = useSecondaryTranslational;
    }

    public boolean isUseSecondaryHeading() {
        return useSecondaryHeading;
    }

    public void setUseSecondaryHeading(boolean useSecondaryHeading) {
        this.useSecondaryHeading = useSecondaryHeading;
    }

    public boolean isUseSecondaryDrive() {
        return useSecondaryDrive;
    }

    public void setUseSecondaryDrive(boolean useSecondaryDrive) {
        this.useSecondaryDrive = useSecondaryDrive;
    }

    public double getTranslationalSwitch() {
        return translationalSwitch;
    }

    public void setTranslationalSwitch(double translationalSwitch) {
        this.translationalSwitch = translationalSwitch;
    }

    public double getTurnHeadingErrorThreshold() {
        return turnHeadingErrorThreshold;
    }

    public void setTurnHeadingErrorThreshold(double turnHeadingErrorThreshold) {
        this.turnHeadingErrorThreshold = turnHeadingErrorThreshold;
    }

    public double getCentripetalScaling() {
        return centripetalScaling;
    }

    public void setCentripetalScaling(double centripetalScaling) {
        this.centripetalScaling = centripetalScaling;
    }

    public boolean isAutomaticHoldEnd() {
        return automaticHoldEnd;
    }

    public void setAutomaticHoldEnd(boolean automaticHoldEnd) {
        this.automaticHoldEnd = automaticHoldEnd;
    }

    public double getMass() {
        return mass;
    }

    public void setMass(double mass) {
        this.mass = mass;
    }

    public double getForwardZeroPowerAcceleration() {
        return forwardZeroPowerAcceleration;
    }

    public void setForwardZeroPowerAcceleration(double forwardZeroPowerAcceleration) {
        this.forwardZeroPowerAcceleration = forwardZeroPowerAcceleration;
    }

    public double getLateralZeroPowerAcceleration() {
        return lateralZeroPowerAcceleration;
    }

    public void setLateralZeroPowerAcceleration(double lateralZeroPowerAcceleration) {
        this.lateralZeroPowerAcceleration = lateralZeroPowerAcceleration;
    }

    public void defaults() {
        coefficientsTranslational.setCoefficients(0.1, 0, 0, 0);
        integralTranslational.setCoefficients(0, 0, 0, 0.015);

        coefficientsHeading.setCoefficients(1, 0, 0, 0.01);

        coefficientsDrive.setCoefficients(0.025, 0, 0.00001, 0.6, 0.01);

        coefficientsSecondaryTranslational.setCoefficients(0.3, 0, 0.01, 0.015);
        integralSecondaryTranslational.setCoefficients(0, 0, 0, 0);

        headingSwitch = Math.PI / 20;
        coefficientsSecondaryHeading.setCoefficients(5, 0, 0.08, 0.01);

        driveSwitch = 20;
        coefficientsSecondaryDrive.setCoefficients(0.02, 0, 0.000005, 0.6, 0.01);
        holdPointTranslationalScaling = 0.45;
        holdPointHeadingScaling = 0.35;

        BEZIER_CURVE_SEARCH_LIMIT = 10;

        useSecondaryTranslational = false;
        useSecondaryHeading = false;
        useSecondaryDrive = false;

        translationalSwitch = 3;
        turnHeadingErrorThreshold = 0.01;
        centripetalScaling = 0.0005;

        automaticHoldEnd = true;
        mass = 10.65;

        forwardZeroPowerAcceleration = -41.278;
        lateralZeroPowerAcceleration = -59.7819;
    }
}