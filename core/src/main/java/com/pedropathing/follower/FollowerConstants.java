package com.pedropathing.follower;

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
     * Translational PIDF coefficients
     * Default Value: new PIDFCoefficients(0.1,0,0,0);
     */
    public PIDFCoefficients coefficientsTranslationalPIDF = new PIDFCoefficients(
            0.1,
            0,
            0,
            0.015);

    /**
     * Translational Integral
     * Default Value: new PIDFCoefficients(0,0,0,0);
     */
    public PIDFCoefficients integralTranslational = new PIDFCoefficients(
            0,
            0,
            0,
            0);

    /**
     * Heading error PIDF coefficients
     * Default Value: new PIDFCoefficients(1,0,0,0);
     */
    public PIDFCoefficients coefficientsHeadingPIDF = new PIDFCoefficients(
            1,
            0,
            0,
            0.01);


    /**
     * Drive PIDF coefficients
     * Default Value: new FilteredPIDFCoefficients(0.025,0,0.00001,0.6,0);
     */
    public FilteredPIDFCoefficients coefficientsDrivePIDF = new FilteredPIDFCoefficients(
            0.025,
            0,
            0.00001,
            0.6,
            0.01);

    /**
     * Secondary translational PIDF coefficients (don't use integral).
     * Default Value: new PIDFCoefficients(0.3, 0, 0.01, 0)
     */
    public PIDFCoefficients coefficientsSecondaryTranslationalPIDF = new PIDFCoefficients(
            0.3,
            0,
            0.01,
            0);

    /**
     * Secondary translational Integral value.
     * Default Value: new PIDFCoefficients(0, 0, 0, 0)
     */
    public PIDFCoefficients integralSecondaryTranslational = new PIDFCoefficients(
            0,
            0,
            0,
            0.015);

    /**
     * The limit at which the heading PIDF switches between the main and secondary heading PIDFs.
     * Default Value: Math.PI / 20
     */
    public double headingPIDFSwitch = Math.PI / 20;

    /**
     * Secondary heading error PIDF coefficients.
     * Default Value: new PIDFCoefficients(5, 0, 0.08, 0)
     */
    public PIDFCoefficients coefficientsSecondaryHeadingPIDF = new PIDFCoefficients(
            5,
            0,
            0.08,
            0.01);

    /**
     * The limit at which the heading PIDF switches between the main and secondary drive PIDFs.
     * Default Value: 20
     */
    public double drivePIDFSwitch = 20;

    /**
     * Secondary drive PIDF coefficients.
     * Default Value: new FilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0)
     */
    public FilteredPIDFCoefficients coefficientsSecondaryDrivePIDF = new FilteredPIDFCoefficients(
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
     * This activates/deactivates the secondary translational PIDF. It takes over at a certain translational error
     *
     * @see #translationalPIDFSwitch
     * Default Value: false
     */
    public boolean useSecondaryTranslationalPIDF = false;

    /**
     * Use the secondary heading PIDF. It takes over at a certain heading error
     *
     * @see #headingPIDFSwitch
     * Default Value: false
     */
    public boolean useSecondaryHeadingPIDF = false;

    /**
     * Use the secondary drive PIDF. It takes over at a certain drive error
     *
     * @see #drivePIDFSwitch
     * Default Value: false
     */
    public boolean useSecondaryDrivePIDF = false;

    /**
     * The limit at which the translational PIDF switches between the main and secondary translational PIDFs,
     * if the secondary PID is active.
     * Default Value: 3
     */
    public double translationalPIDFSwitch = 3;

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

    public FollowerConstants translationalPIDFCoefficients(PIDFCoefficients translationalPIDFCoefficients) {
        this.coefficientsTranslationalPIDF = translationalPIDFCoefficients;
        return this;
    }

    public FollowerConstants headingPIDFCoefficients(PIDFCoefficients headingPIDFCoefficients) {
        this.coefficientsHeadingPIDF = headingPIDFCoefficients;
        return this;
    }

    public FollowerConstants drivePIDFCoefficients(FilteredPIDFCoefficients drivePIDFCoefficients) {
        this.coefficientsDrivePIDF = drivePIDFCoefficients;
        return this;
    }

    public FollowerConstants secondaryTranslationalPIDFCoefficients(PIDFCoefficients secondaryTranslationalPIDFCoefficients) {
        this.coefficientsSecondaryTranslationalPIDF = secondaryTranslationalPIDFCoefficients;
        useSecondaryTranslationalPIDF = true;
        return this;
    }

    public FollowerConstants headingPIDFSwitch(double headingPIDFSwitch) {
        this.headingPIDFSwitch = headingPIDFSwitch;
        return this;
    }

    public FollowerConstants secondaryHeadingPIDFCoefficients(PIDFCoefficients secondaryHeadingPIDFCoefficients) {
        this.coefficientsSecondaryHeadingPIDF = secondaryHeadingPIDFCoefficients;
        useSecondaryHeadingPIDF = true;
        return this;
    }

    public FollowerConstants drivePIDFSwitch(double drivePIDFSwitch) {
        this.drivePIDFSwitch = drivePIDFSwitch;
        return this;
    }

    public FollowerConstants secondaryDrivePIDFCoefficients(FilteredPIDFCoefficients secondaryDrivePIDFCoefficients) {
        this.coefficientsSecondaryDrivePIDF = secondaryDrivePIDFCoefficients;
        useSecondaryDrivePIDF = true;
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

    public FollowerConstants useSecondaryTranslationalPIDF(boolean useSecondaryTranslationalPIDF) {
        this.useSecondaryTranslationalPIDF = useSecondaryTranslationalPIDF;
        return this;
    }

    public FollowerConstants useSecondaryHeadingPIDF(boolean useSecondaryHeadingPIDF) {
        this.useSecondaryHeadingPIDF = useSecondaryHeadingPIDF;
        return this;
    }

    public FollowerConstants useSecondaryDrivePIDF(boolean useSecondaryDrivePIDF) {
        this.useSecondaryDrivePIDF = useSecondaryDrivePIDF;
        return this;
    }

    public FollowerConstants translationalPIDFSwitch(double translationalPIDFSwitch) {
        this.translationalPIDFSwitch = translationalPIDFSwitch;
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

    public PIDFCoefficients getCoefficientsTranslationalPIDF() {
        return coefficientsTranslationalPIDF;
    }

    public void setCoefficientsTranslationalPIDF(PIDFCoefficients coefficientsTranslationalPIDF) {
        this.coefficientsTranslationalPIDF = coefficientsTranslationalPIDF;
    }

    public PIDFCoefficients getIntegralTranslational() {
        return integralTranslational;
    }

    public void setIntegralTranslational(PIDFCoefficients integralTranslational) {
        this.integralTranslational = integralTranslational;
    }

    public PIDFCoefficients getCoefficientsHeadingPIDF() {
        return coefficientsHeadingPIDF;
    }

    public void setCoefficientsHeadingPIDF(PIDFCoefficients coefficientsHeadingPIDF) {
        this.coefficientsHeadingPIDF = coefficientsHeadingPIDF;
    }
    public FilteredPIDFCoefficients getCoefficientsDrivePIDF() {
        return coefficientsDrivePIDF;
    }

    public void setCoefficientsDrivePIDF(FilteredPIDFCoefficients coefficientsDrivePIDF) {
        this.coefficientsDrivePIDF = coefficientsDrivePIDF;
    }

    public PIDFCoefficients getCoefficientsSecondaryTranslationalPIDF() {
        return coefficientsSecondaryTranslationalPIDF;
    }

    public void setCoefficientsSecondaryTranslationalPIDF(PIDFCoefficients coefficientsSecondaryTranslationalPIDF) {
        this.coefficientsSecondaryTranslationalPIDF = coefficientsSecondaryTranslationalPIDF;
        useSecondaryTranslationalPIDF = true;
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

    public double getHeadingPIDFSwitch() {
        return headingPIDFSwitch;
    }

    public void setHeadingPIDFSwitch(double headingPIDFSwitch) {
        this.headingPIDFSwitch = headingPIDFSwitch;
    }

    public PIDFCoefficients getCoefficientsSecondaryHeadingPIDF() {
        return coefficientsSecondaryHeadingPIDF;
    }

    public void setCoefficientsSecondaryHeadingPIDF(PIDFCoefficients coefficientsSecondaryHeadingPIDF) {
        this.coefficientsSecondaryHeadingPIDF = coefficientsSecondaryHeadingPIDF;
        useSecondaryHeadingPIDF = true;
    }

    public double getDrivePIDFSwitch() {
        return drivePIDFSwitch;
    }

    public void setDrivePIDFSwitch(double drivePIDFSwitch) {
        this.drivePIDFSwitch = drivePIDFSwitch;
    }

    public FilteredPIDFCoefficients getCoefficientsSecondaryDrivePIDF() {
        return coefficientsSecondaryDrivePIDF;
    }

    public void setCoefficientsSecondaryDrivePIDF(FilteredPIDFCoefficients coefficientsSecondaryDrivePIDF) {
        this.coefficientsSecondaryDrivePIDF = coefficientsSecondaryDrivePIDF;
        useSecondaryDrivePIDF = true;
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

    public boolean isUseSecondaryTranslationalPIDF() {
        return useSecondaryTranslationalPIDF;
    }

    public void setUseSecondaryTranslationalPIDF(boolean useSecondaryTranslationalPIDF) {
        this.useSecondaryTranslationalPIDF = useSecondaryTranslationalPIDF;
    }

    public boolean isUseSecondaryHeadingPIDF() {
        return useSecondaryHeadingPIDF;
    }

    public void setUseSecondaryHeadingPIDF(boolean useSecondaryHeadingPIDF) {
        this.useSecondaryHeadingPIDF = useSecondaryHeadingPIDF;
    }

    public boolean isUseSecondaryDrivePIDF() {
        return useSecondaryDrivePIDF;
    }

    public void setUseSecondaryDrivePIDF(boolean useSecondaryDrivePIDF) {
        this.useSecondaryDrivePIDF = useSecondaryDrivePIDF;
    }

    public double getTranslationalPIDFSwitch() {
        return translationalPIDFSwitch;
    }

    public void setTranslationalPIDFSwitch(double translationalPIDFSwitch) {
        this.translationalPIDFSwitch = translationalPIDFSwitch;
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
        coefficientsTranslationalPIDF.setCoefficients(0.1, 0, 0, 0);
        integralTranslational.setCoefficients(0, 0, 0, 0.015);

        coefficientsHeadingPIDF.setCoefficients(1, 0, 0, 0.01);

        coefficientsDrivePIDF.setCoefficients(0.025, 0, 0.00001, 0.6, 0.01);

        coefficientsSecondaryTranslationalPIDF.setCoefficients(0.3, 0, 0.01, 0.015);
        integralSecondaryTranslational.setCoefficients(0, 0, 0, 0);

        headingPIDFSwitch = Math.PI / 20;
        coefficientsSecondaryHeadingPIDF.setCoefficients(5, 0, 0.08, 0.01);

        drivePIDFSwitch = 20;
        coefficientsSecondaryDrivePIDF.setCoefficients(0.02, 0, 0.000005, 0.6, 0.01);
        holdPointTranslationalScaling = 0.45;
        holdPointHeadingScaling = 0.35;

        BEZIER_CURVE_SEARCH_LIMIT = 10;

        useSecondaryTranslationalPIDF = false;
        useSecondaryHeadingPIDF = false;
        useSecondaryDrivePIDF = false;

        translationalPIDFSwitch = 3;
        turnHeadingErrorThreshold = 0.01;
        centripetalScaling = 0.0005;

        automaticHoldEnd = true;
        mass = 10.65;

        forwardZeroPowerAcceleration = -41.278;
        lateralZeroPowerAcceleration = -59.7819;
    }
}