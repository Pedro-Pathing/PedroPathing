package com.pedropathing.revhub.localizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RevHubIMU implements CustomIMU {
    private IMU imu;

    /**
     * Initializes the IMU using the hardwareMap and hubOrientation.
     * @param hardwareMap the hardware map
     * @param hardwareMapName the name of the hardware map
     * @param hubOrientation the hub orientation
     */
    @Override
    public void initialize(HardwareMap hardwareMap, String hardwareMapName, RevHubOrientationOnRobot hubOrientation) {
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
