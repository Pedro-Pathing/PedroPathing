package com.pedropathing.revhub.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface CustomIMU {
    /**
     * Initializes the IMU using the hardwareMap and hubOrientation.
     */
    void initialize(HardwareMap hardwareMap, String hardwareMapName);

    /**
     * Gets the IMU's reading for the heading of the robot in radians
     * @return the heading of the robot in radians
     */
    double getHeading();

    /**
     * Resets the IMU's yaw to 0.
     */
    void resetYaw();

    /**
     * Gets the IMU's angular velocity about the vertical axis in radians per second.
     * <p>
     * The default returns {@link Double#NaN}, which disables angular-velocity-based filtering.
     * Implementations that can measure rotation rate should override this.
     */
    default double getAngularVelocity() {
        return Double.NaN;
    }
}


