package com.pedropathing.control;

/**
 * This is the LowPassFilter class. This class implements a simple low-pass filter to smooth out noisy data.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class LowPassFilter implements NoiseFilter {
    private final double alpha;
    private double state;
    private boolean initialized;

    /**
     * Constructs a LowPassFilter with the specified alpha value.
     * @param alpha The smoothing factor (0 [less than] alpha [less than] 1). A higher alpha means more smoothing.
     */
    public LowPassFilter(double alpha) {
        if (alpha <= 0 || alpha >= 1) {
            throw new IllegalArgumentException("Alpha must be between 0 and 1 (exclusive).");
        }
        this.alpha = alpha;
        this.initialized = false;
    }

    /**
     * Updates the filter with new data.
     * @param updateData The new data to update the filter with.
     * @param updateProjection The projection of the new data (not used in LowPassFilter).
     */
    @Override
    public void update(double updateData, double updateProjection) {
        if (!initialized) {
            state = updateData;
            initialized = true;
        } else {
            state = alpha * updateData + (1 - alpha) * state;
        }
    }

    /**
     * Gets the current state of the filter.
     * @return The current state of the filter.
     */
    @Override
    public double getState() {
        return state;
    }

    /**
     * Resets the filter to a specified state.
     * @param startState The state to reset the filter to.
     * @param startVariance The variance to reset the filter to (not used in LowPassFilter).
     * @param startGain The gain to reset the filter to (not used in LowPassFilter).
     */
    @Override
    public void reset(double startState, double startVariance, double startGain) {
        this.state = startState;
        this.initialized = true;
    }
}
