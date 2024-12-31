package com.pedropathing.localization.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * This is the ThreeWheelIMUConstants class. It holds many constants and parameters for the Three Wheel + IMU Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

public class ThreeWheelIMUConstants {

    /** The number of inches per tick of the encoder for forward movement
     * @value Default Value: .001989436789 */
    public static double forwardTicksToInches = .001989436789;

    /** The number of inches per tick of the encoder for lateral movement (strafing)
     * @value Default Value: .001989436789 */
    public static double strafeTicksToInches = .001989436789;

    /** The number of inches per tick of the encoder for turning
     * @value Default Value: .001989436789 */
    public static double turnTicksToInches = .001989436789;

    /** The X, Y, and Rotation Offset of the Left Encoder (Deadwheel) from the center of the robot
     * @value Default Value: 0, 1, 0deg */
    public static Pose leftEncoderPose = new Pose(0, 1, Math.toRadians(0));

    /** The X, Y, and Rotation Offset of the Right Encoder (Deadwheel) from the center of the robot
     * @value Default Value: 0, -1, 0deg */
    public static Pose rightEncoderPose = new Pose(0, -1, Math.toRadians(0));

    /** The X, Y, and Rotation Offset of the Strafe Encoder (Deadwheel) from the center of the robot
     * @value Default Value: -2.5, 0, 90deg */
    public static Pose strafeEncoderPose = new Pose(-2.5, 0, Math.toRadians(90));

    /** The Hardware Map Name of the IMU (built-in IMU will be Port 0, "imu")
     * @value Default Value: "imu" */
    public static String IMU_HardwareMapName = "imu";

    /** The name of the Left Encoder in the hardware map (name of the motor port it is plugged into)
     * @value Default Value: "leftFront" */
    public static String leftEncoder_HardwareMapName = "leftFront";

    /** The name of the Right Encoder in the hardware map (name of the motor port it is plugged into)
     * @value Default Value: "rightRear" */
    public static String rightEncoder_HardwareMapName = "rightRear";

    /** The name of the Strafe Encoder in the hardware map (name of the motor port it is plugged into)
     * @value Default Value: Default Value: "rightFront" */
    public static String strafeEncoder_HardwareMapName = "rightFront";

    /** The Orientation of the Control Hub (for IMU) on the Robot
     * @value Default Value: new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT) */
    public static RevHubOrientationOnRobot IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    /** The direction of the Left Encoder
     * @value Default Value: Encoder.REVERSE */
    public static double leftEncoderDirection = Encoder.REVERSE;

    /** The direction of the Right Encoder
     * @value Default Value: Encoder.FORWARD */
    public static double rightEncoderDirection = Encoder.REVERSE;

    /** The direction of the Strafe Encoder
     * @value Default Value: Encoder.FORWARD */
    public static double strafeEncoderDirection = Encoder.FORWARD;

}
