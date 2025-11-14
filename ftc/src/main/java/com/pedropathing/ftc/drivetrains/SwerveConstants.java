package com.pedropathing.ftc.drivetrains;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SwerveConstants {
    public String frontLeftMotorName = "frontLeftDrive";
    public String frontLeftServoName = "frontLeftTurnServo";
    public String frontLeftEncoderName = "frontLeftTurnEncoder";

    public String frontRightMotorName = "frontRightDrive";
    public String frontRightServoName = "frontRightTurnServo";
    public String frontRightEncoderName = "frontRightTurnEncoder";

    public String backLeftMotorName = "backLeftDrive";
    public String backLeftServoName = "backLeftTurnServo";
    public String backLeftEncoderName = "backLeftTurnEncoder";

    public String backRightMotorName = "backRightDrive";
    public String backRightServoName = "backRightTurnServo";
    public String backRightEncoderName = "backRightTurnEncoder";

    //TODO: Change PID coefficients based on your config
    public PIDFCoefficients frontLeftTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
    public PIDFCoefficients frontRightTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
    public PIDFCoefficients backLeftTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
    public PIDFCoefficients backRightTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);

    //TODO: Reverse motors if needed
    public DcMotorEx.Direction frontLeftMotorDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorEx.Direction frontRightMotorDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorEx.Direction backLeftMotorDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorEx.Direction backRightMotorDirection = DcMotorSimple.Direction.FORWARD;

    //TODO: Reverse servos if needed
    public boolean frontLeftServoReversed = false;
    public boolean frontRightServoReversed = false;
    public boolean backLeftServoReversed = false;
    public boolean backRightServoReversed = false;

    //TODO: These are the negatives of the reported angle of each pod when facing forward, in degrees
    public double frontLeftPodAngleOffsetDeg = 0.0;
    public double frontRightPodAngleOffsetDeg = 0.0;
    public double backLeftPodAngleOffsetDeg = 0.0;
    public double backRightPodAngleOffsetDeg = 0.0;

    //TODO: distance from the center of the robot to each pod
    //if you're swerve is square, you can leave these be
    // hopefully these should be +- the same x, y, but just in case
    //units don't matter, they'll be normalized
    //positive x is right, positive y is forward, as with joysticks
    public double[] frontLeftPodXYOffsets = new double[] {-1, 1};
    public double[] frontRightPodXYOffsets = new double[] {1, 1};
    public double[] backLeftPodXYOffsets = new double[] {-1, -1};
    public double[] backRightPodXYOffsets = new double[] {1, -1};


}
