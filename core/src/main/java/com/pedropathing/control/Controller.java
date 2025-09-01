package com.pedropathing.control;

/**
 * This is the Controller interface. This interface defines the methods that a controller must implement.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public interface Controller {
    /**
     * This is the Coefficients interface. This interface defines the methods that a coefficients class must implement.
     */
    interface Coefficients {
        /**
         * This creates a new controller with the given coefficients.
         * @return A new controller with the given coefficients.
         */
        Controller create();

        /**
         * This sets the coefficients of the controller.
         * @param coefficients The coefficients to set.
         */
        void setCoefficients(double... coefficients);
    }

    /**
     * This returns the current coefficients of the controller.
     * @return The current coefficients of the controller.
     */
    Coefficients getCoefficients();

    /**
     * This sets the coefficients of the controller.
     * @param coefficients The coefficients to set.
     */
    void setCoefficients(Coefficients coefficients);

    /**
     * This updates the position of the controller.
     * @param update The new position to update the controller with.
     */
    void updatePosition(double update);

    /**
     * This updates the error of the controller.
     * @param error The new error to update the controller with.
     */
    void updateError(double error);

    /**
     * This resets the controller.
     */
    void reset();

    /**
     * This runs the controller and returns the output.
     * @return The output of the controller.
     */
    double run();

    /**
     * This updates the feedforward input of the controller.
     * @param feedforwardInput The new feedforward input to update the controller with.
     */
    void updateFeedForwardInput(double feedforwardInput);

    /**
     * This sets the target position of the controller.
     * @param set The new target position to set the controller to.
     */
    void setTargetPosition(double set);

    /**
     * This returns the target position of the controller.
     * @return The target position of the controller.
     */
    double getTargetPosition();

    /**
     * This returns the current error of the controller.
     * @return The current error of the controller.
     */
    double getError();
}
