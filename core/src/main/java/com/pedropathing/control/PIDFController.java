package com.pedropathing.control;

/**
 * This is the PIDFController class. This class handles the running of PIDFs. PIDF stands for
 * proportional, integral, derivative, and feedforward. PIDFs take the error of a system as an input.
 * Coefficients multiply into the error, the integral of the error, the derivative of the error, and
 * a feedforward value. Then, these values are added up and returned. In this way, error in the
 * system is corrected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/5/2024
 */
public class PIDFController implements Controller<PIDFCoefficients> {
    private PIDFCoefficients coefficients;

    private double previousError;
    private double error;
    private double targetPosition;
    private double errorIntegral;
    private double errorDerivative;

    private long previousUpdateTimeNano;
    
    /**
     * This creates a new PIDFController from a CustomPIDFCoefficients.
     *
     * @param set the coefficients to use.
     */
    public PIDFController(PIDFCoefficients set) {
        setCoefficients(set);
        reset();
    }

    /**
     * This takes the current error and runs the PIDF on it.
     *
     * @return this returns the value of the PIDF from the current error.
     */
    public double run(double error) {
        updateError(error);
        return error * P() + errorDerivative * D() + errorIntegral * I() + F() * Math.signum(error);
    }

    /**
     * As opposed to updating position against a target position, this just sets the error to some
     * specified value.
     *
     * @param error The error specified.
     */
    public void updateError(double error) {
        previousError = this.error;
        this.error = error;
        long nanoTime = System.nanoTime();
        
        long deltaTimeNano = nanoTime - previousUpdateTimeNano;
        previousUpdateTimeNano = nanoTime;

        errorIntegral += error * (deltaTimeNano / 1e9);
        errorDerivative = (error - previousError) / (deltaTimeNano / 1e9);
    }

    /**
     * This resets all the PIDF's error and position values, as well as the time stamps.
     */
    public void reset() {
        previousError = 0;
        error = 0;
        targetPosition = 0;
        errorIntegral = 0;
        errorDerivative = 0;
        previousUpdateTimeNano = System.nanoTime();
    }

    /**
     * This is used to set the target position if the PIDF is being run with current position and
     * target position inputs rather than error inputs.
     *
     * @param set this sets the target position.
     */
    public void setTargetPosition(double set) {
        targetPosition = set;
    }

    /**
     * This returns the target position of the PIDF.
     *
     * @return this returns the target position.
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * This is used to set the coefficients of the PIDF.
     *
     * @param set the coefficients that the PIDF will use.
     */
    @Override
    public void setCoefficients(PIDFCoefficients set) {
        coefficients = set;
    }

    /**
     * This returns the PIDF's current coefficients.
     *
     * @return this returns the current coefficients.
     */
    public PIDFCoefficients getCoefficients() {
        return coefficients;
    }

    /**
     * This sets the proportional (P) coefficient of the PIDF only.
     *
     * @param set this sets the P coefficient.
     */
    public void setP(double set) {
        coefficients.P = set;
    }

    /**
     * This returns the proportional (P) coefficient of the PIDF.
     *
     * @return this returns the P coefficient.
     */
    public double P() {
        return coefficients.P;
    }

    /**
     * This sets the integral (I) coefficient of the PIDF only.
     *
     * @param set this sets the I coefficient.
     */
    public void setI(double set) {
        coefficients.I = set;
    }

    /**
     * This returns the integral (I) coefficient of the PIDF.
     *
     * @return this returns the I coefficient.
     */
    public double I() {
        return coefficients.I;
    }

    /**
     * This sets the derivative (D) coefficient of the PIDF only.
     *
     * @param set this sets the D coefficient.
     */
    public void setD(double set) {
        coefficients.D = set;
    }

    /**
     * This returns the derivative (D) coefficient of the PIDF.
     *
     * @return this returns the D coefficient.
     */
    public double D() {
        return coefficients.D;
    }

    /**
     * This sets the feedforward (F) constant of the PIDF only.
     *
     * @param set this sets the F constant.
     */
    public void setF(double set) {
        coefficients.F = set;
    }

    /**
     * This returns the feedforward (F) constant of the PIDF.
     *
     * @return this returns the F constant.
     */
    public double F() {
        return coefficients.F;
    }

    /**
     * This returns the current error of the PIDF.
     *
     * @return this returns the error.
     */
    public double getError() {
        return error;
    }

    /**
     * This returns the current derivative of the error.
     * @return the derivative
     */
    public double getErrorDerivative() {
        return errorDerivative;
    }
}
