package com.pedropathing.control;

/**
 * This is the SquIDController class. This class allows for using a SquID controller instead of a PID controller.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class SquIDController implements Controller {
    private SquIDCoefficients coefficients;
    private double error;
    private double targetPosition;
    private double feedforwardInput;

    public SquIDController(SquIDCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    @Override
    public SquIDCoefficients getCoefficients() {
        return coefficients;
    }

    @Override
    public void setCoefficients(Coefficients coefficients) {
        if (!(coefficients instanceof SquIDCoefficients)) {
            throw new IllegalArgumentException("Expected SquIDCoefficients, got " + coefficients.getClass().getSimpleName());
        }
        this.coefficients = (SquIDCoefficients) coefficients;
    }

    @Override
    public void updatePosition(double update) {
        error = targetPosition - update;
    }

    @Override
    public void updateError(double error) {
        this.error = error;
    }

    @Override
    public void reset() {
        error = 0;
        targetPosition = 0;
        feedforwardInput = 0;
    }

    @Override
    public double run() {
        return coefficients.getP() * Math.sqrt(Math.abs(error)) * Math.signum(error) + coefficients.getF() * feedforwardInput;
    }

    @Override
    public void updateFeedForwardInput(double feedforwardInput) {
        this.feedforwardInput = feedforwardInput;
    }

    @Override
    public void setTargetPosition(double set) {
        targetPosition = set;
    }

    @Override
    public double getTargetPosition() {
        return targetPosition;
    }

    @Override
    public double getError() {
        return error;
    }
}
