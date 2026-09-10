package com.pedropathing.revhub.localizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RevHubIMU implements CustomIMU {
    private IMU imu;
    private final RevHubOrientationOnRobot hubOrientation;

    public RevHubIMU(RevHubOrientationOnRobot hubOrientation) {
        this.hubOrientation = hubOrientation;
    }

    /**
     * Initializes the IMU using the hardwareMap and hubOrientation.
     * @param hardwareMap the hardware map
     * @param hardwareMapName the name of the hardware map
     */
    @Override
    public void initialize(HardwareMap hardwareMap, String hardwareMapName) {
        imu = hardwareMap.get(IMU.class, hardwareMapName);
        imu.initialize(new IMU.Parameters(hubOrientation));
    }

    /**
     * Gets the IMU's reading for the heading of the robot in radians
     * @return the heading of the robot in radians
     */
    @Override
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Resets the IMU's yaw to 0.
     */
    @Override
    public void resetYaw() {
        imu.resetYaw();
    }
}
