package com.pedropathing.ftc.drivetrains;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SwerveConstants {
    public String leftFrontMotorName = "frontLeftDrive";
    public String leftFrontServoName = "frontLeftTurnServo";
    public String leftFrontEncoderName = "frontLeftTurnEncoder";

    public String rightFrontMotorName = "frontRightDrive";
    public String rightFrontServoName = "frontRightTurnServo";
    public String rightFrontEncoderName = "frontRightTurnEncoder";

    public String leftRearMotorName = "backLeftDrive";
    public String leftRearServoName = "backLeftTurnServo";
    public String leftRearEncoderName = "backLeftTurnEncoder";

    public String rightRearMotorName = "backRightDrive";
    public String rightRearServoName = "backRightTurnServo";
    public String rightRearEncoderName = "backRightTurnEncoder";

    //TODO: Change PID coefficients based on your config
    public PIDFCoefficients leftFrontTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
    public PIDFCoefficients rightFrontTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
    public PIDFCoefficients leftRearTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
    public PIDFCoefficients rightRearTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);

    //TODO: Reverse motors if needed
    public DcMotorEx.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorEx.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorEx.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorEx.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    //TODO: Reverse servos if needed
    public boolean leftFrontServoReversed = false;
    public boolean rightFrontServoReversed = false;
    public boolean leftRearServoReversed = false;
    public boolean rightRearServoReversed = false;

    //TODO: These are the negatives of the reported angle of each pod when facing forward, in degrees
    public double leftFrontPodAngleOffsetDeg = 0.0;
    public double rightFrontPodAngleOffsetDeg = 0.0;
    public double leftRearPodAngleOffsetDeg = 0.0;
    public double rightRearPodAngleOffsetDeg = 0.0;

    //TODO: distance from the center of the robot to each pod
    //if you're swerve is square, you can leave these be
    // hopefully these should be +- the same x, y, but just in case
    //units don't matter, they'll be normalized
    //positive x is right, positive y is forward, as with joysticks
    public double[] leftFrontPodXYOffsets = new double[] {-1, 1};
    public double[] rightFrontPodXYOffsets = new double[] {1, 1};
    public double[] leftRearPodXYOffsets = new double[] {-1, -1};
    public double[] rightRearPodXYOffsets = new double[] {1, -1};

    public SwerveConstants() {
        defaults();
    }


    public SwerveConstants leftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
        return this;
    }

    public SwerveConstants leftFrontServoName(String leftFrontServoName) {
        this.leftFrontServoName = leftFrontServoName;
        return this;
    }

    public SwerveConstants leftFrontEncoderName(String leftFrontEncoderName) {
        this.leftFrontEncoderName = leftFrontEncoderName;
        return this;
    }

    public SwerveConstants rightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    public SwerveConstants rightFrontServoName(String rightFrontServoName) {
        this.rightFrontServoName = rightFrontServoName;
        return this;
    }

    public SwerveConstants rightFrontEncoderName(String rightFrontEncoderName) {
        this.rightFrontEncoderName = rightFrontEncoderName;
        return this;
    }

    public SwerveConstants leftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
        return this;
    }

    public SwerveConstants leftRearServoName(String leftRearServoName) {
        this.leftRearServoName = leftRearServoName;
        return this;
    }

    public SwerveConstants leftRearEncoderName(String leftRearEncoderName) {
        this.leftRearEncoderName = leftRearEncoderName;
        return this;
    }

    public SwerveConstants rightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
        return this;
    }

    public SwerveConstants rightRearServoName(String rightRearServoName) {
        this.rightRearServoName = rightRearServoName;
        return this;
    }

    public SwerveConstants rightRearEncoderName(String rightRearEncoderName) {
        this.rightRearEncoderName = rightRearEncoderName;
        return this;
    }

    public SwerveConstants leftFrontTurnPID(PIDFCoefficients leftFrontTurnPID) {
        this.leftFrontTurnPID = leftFrontTurnPID;
        return this;
    }

    public SwerveConstants rightFrontTurnPID(PIDFCoefficients rightFrontTurnPID) {
        this.rightFrontTurnPID = rightFrontTurnPID;
        return this;
    }

    public SwerveConstants leftRearTurnPID(PIDFCoefficients leftRearTurnPID) {
        this.leftRearTurnPID = leftRearTurnPID;
        return this;
    }

    public SwerveConstants rightRearTurnPID(PIDFCoefficients rightRearTurnPID) {
        this.rightRearTurnPID = rightRearTurnPID;
        return this;
    }

    public SwerveConstants leftFrontMotorDirection(DcMotorEx.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
        return this;
    }

    public SwerveConstants rightFrontMotorDirection(DcMotorEx.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
        return this;
    }

    public SwerveConstants leftRearMotorDirection(DcMotorEx.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
        return this;
    }

    public SwerveConstants rightRearMotorDirection(DcMotorEx.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
        return this;
    }

    public SwerveConstants leftFrontServoReversed(boolean leftFrontServoReversed) {
        this.leftFrontServoReversed = leftFrontServoReversed;
        return this;
    }

    public SwerveConstants rightFrontServoReversed(boolean rightFrontServoReversed) {
        this.rightFrontServoReversed = rightFrontServoReversed;
        return this;
    }

    public SwerveConstants leftRearServoReversed(boolean leftRearServoReversed) {
        this.leftRearServoReversed = leftRearServoReversed;
        return this;
    }

    public SwerveConstants rightRearServoReversed(boolean rightRearServoReversed) {
        this.rightRearServoReversed = rightRearServoReversed;
        return this;
    }

    public SwerveConstants leftFrontPodAngleOffsetDeg(double leftFrontPodAngleOffsetDeg) {
        this.leftFrontPodAngleOffsetDeg = leftFrontPodAngleOffsetDeg;
        return this;
    }

    public SwerveConstants rightFrontPodAngleOffsetDeg(double rightFrontPodAngleOffsetDeg) {
        this.rightFrontPodAngleOffsetDeg = rightFrontPodAngleOffsetDeg;
        return this;
    }

    public SwerveConstants leftRearPodAngleOffsetDeg(double leftRearPodAngleOffsetDeg) {
        this.leftRearPodAngleOffsetDeg = leftRearPodAngleOffsetDeg;
        return this;
    }

    public SwerveConstants rightRearPodAngleOffsetDeg(double rightRearPodAngleOffsetDeg) {
        this.rightRearPodAngleOffsetDeg = rightRearPodAngleOffsetDeg;
        return this;
    }

    public SwerveConstants leftFrontPodXYOffsets(double[] leftFrontPodXYOffsets) {
        this.leftFrontPodXYOffsets = leftFrontPodXYOffsets;
        return this;
    }

    public SwerveConstants rightFrontPodXYOffsets(double[] rightFrontPodXYOffsets) {
        this.rightFrontPodXYOffsets = rightFrontPodXYOffsets;
        return this;
    }

    public SwerveConstants leftRearPodXYOffsets(double[] leftRearPodXYOffsets) {
        this.leftRearPodXYOffsets = leftRearPodXYOffsets;
        return this;
    }

    public SwerveConstants rightRearPodXYOffsets(double[] rightRearPodXYOffsets) {
        this.rightRearPodXYOffsets = rightRearPodXYOffsets;
        return this;
    }

    public String getLeftFrontMotorName() {
        return leftFrontMotorName;
    }

    public String getLeftFrontServoName() {
        return leftFrontServoName;
    }

    public String getLeftFrontEncoderName() {
        return leftFrontEncoderName;
    }

    public String getRightFrontMotorName() {
        return rightFrontMotorName;
    }

    public String getRightFrontServoName() {
        return rightFrontServoName;
    }

    public String getRightFrontEncoderName() {
        return rightFrontEncoderName;
    }

    public String getLeftRearMotorName() {
        return leftRearMotorName;
    }

    public String getLeftRearServoName() {
        return leftRearServoName;
    }

    public String getLeftRearEncoderName() {
        return leftRearEncoderName;
    }

    public String getRightRearMotorName() {
        return rightRearMotorName;
    }

    public String getRightRearServoName() {
        return rightRearServoName;
    }

    public String getRightRearEncoderName() {
        return rightRearEncoderName;
    }

    public PIDFCoefficients getLeftFrontTurnPID() {
        return leftFrontTurnPID;
    }

    public PIDFCoefficients getRightFrontTurnPID() {
        return rightFrontTurnPID;
    }

    public PIDFCoefficients getLeftRearTurnPID() {
        return leftRearTurnPID;
    }

    public PIDFCoefficients getRightRearTurnPID() {
        return rightRearTurnPID;
    }

    public DcMotorEx.Direction getLeftFrontMotorDirection() {
        return leftFrontMotorDirection;
    }

    public DcMotorEx.Direction getRightFrontMotorDirection() {
        return rightFrontMotorDirection;
    }

    public DcMotorEx.Direction getLeftRearMotorDirection() {
        return leftRearMotorDirection;
    }

    public DcMotorEx.Direction getRightRearMotorDirection() {
        return rightRearMotorDirection;
    }

    public boolean getLeftFrontServoReversed() {
        return leftFrontServoReversed;
    }

    public boolean getRightFrontServoReversed() {
        return rightFrontServoReversed;
    }

    public boolean getLeftRearServoReversed() {
        return leftRearServoReversed;
    }

    public boolean getRightRearServoReversed() {
        return rightRearServoReversed;
    }

    public double getLeftFrontPodAngleOffsetDeg() {
        return leftFrontPodAngleOffsetDeg;
    }

    public double getRightFrontPodAngleOffsetDeg() {
        return rightFrontPodAngleOffsetDeg;
    }

    public double getLeftRearPodAngleOffsetDeg() {
        return leftRearPodAngleOffsetDeg;
    }

    public double getRightRearPodAngleOffsetDeg() {
        return rightRearPodAngleOffsetDeg;
    }

    public double[] getLeftFrontPodXYOffsets() {
        return leftFrontPodXYOffsets;
    }

    public double[] getRightFrontPodXYOffsets() {
        return rightFrontPodXYOffsets;
    }

    public double[] getLeftRearPodXYOffsets() {
        return leftRearPodXYOffsets;
    }

    public double[] getRightRearPodXYOffsets() {
        return rightRearPodXYOffsets;
    }

    public void setLeftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
    }

    public void setLeftFrontServoName(String leftFrontServoName) {
        this.leftFrontServoName = leftFrontServoName;
    }

    public void setLeftFrontEncoderName(String leftFrontEncoderName) {
        this.leftFrontEncoderName = leftFrontEncoderName;
    }

    public void setRightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
    }

    public void setRightFrontServoName(String rightFrontServoName) {
        this.rightFrontServoName = rightFrontServoName;
    }

    public void setRightFrontEncoderName(String rightFrontEncoderName) {
        this.rightFrontEncoderName = rightFrontEncoderName;
    }

    public void setLeftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
    }

    public void setLeftRearServoName(String leftRearServoName) {
        this.leftRearServoName = leftRearServoName;
    }

    public void setLeftRearEncoderName(String leftRearEncoderName) {
        this.leftRearEncoderName = leftRearEncoderName;
    }

    public void setRightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
    }

    public void setRightRearServoName(String rightRearServoName) {
        this.rightRearServoName = rightRearServoName;
    }

    public void setRightRearEncoderName(String rightRearEncoderName) {
        this.rightRearEncoderName = rightRearEncoderName;
    }

    public void setLeftFrontTurnPID(PIDFCoefficients leftFrontTurnPID) {
        this.leftFrontTurnPID = leftFrontTurnPID;
    }

    public void setRightFrontTurnPID(PIDFCoefficients rightFrontTurnPID) {
        this.rightFrontTurnPID = rightFrontTurnPID;
    }

    public void setLeftRearTurnPID(PIDFCoefficients leftRearTurnPID) {
        this.leftRearTurnPID = leftRearTurnPID;
    }

    public void setRightRearTurnPID(PIDFCoefficients rightRearTurnPID) {
        this.rightRearTurnPID = rightRearTurnPID;
    }

    public void setLeftFrontMotorDirection(DcMotorEx.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
    }

    public void setRightFrontMotorDirection(DcMotorEx.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
    }

    public void setLeftRearMotorDirection(DcMotorEx.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
    }

    public void setRightRearMotorDirection(DcMotorEx.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
    }

    public void setLeftFrontServoReversed(boolean leftFrontServoReversed) {
        this.leftFrontServoReversed = leftFrontServoReversed;
    }

    public void setRightFrontServoReversed(boolean rightFrontServoReversed) {
        this.rightFrontServoReversed = rightFrontServoReversed;
    }

    public void setLeftRearServoReversed(boolean leftRearServoReversed) {
        this.leftRearServoReversed = leftRearServoReversed;
    }

    public void setRightRearServoReversed(boolean rightRearServoReversed) {
        this.rightRearServoReversed = rightRearServoReversed;
    }

    public void setLeftFrontPodAngleOffsetDeg(double leftFrontPodAngleOffsetDeg) {
        this.leftFrontPodAngleOffsetDeg = leftFrontPodAngleOffsetDeg;
    }

    public void setRightFrontPodAngleOffsetDeg(double rightFrontPodAngleOffsetDeg) {
        this.rightFrontPodAngleOffsetDeg = rightFrontPodAngleOffsetDeg;
    }

    public void setLeftRearPodAngleOffsetDeg(double leftRearPodAngleOffsetDeg) {
        this.leftRearPodAngleOffsetDeg = leftRearPodAngleOffsetDeg;
    }

    public void setRightRearPodAngleOffsetDeg(double rightRearPodAngleOffsetDeg) {
        this.rightRearPodAngleOffsetDeg = rightRearPodAngleOffsetDeg;
    }

    public void setLeftFrontPodXYOffsets(double[] leftFrontPodXYOffsets) {
        this.leftFrontPodXYOffsets = leftFrontPodXYOffsets;
    }

    public void setRightFrontPodXYOffsets(double[] rightFrontPodXYOffsets) {
        this.rightFrontPodXYOffsets = rightFrontPodXYOffsets;
    }

    public void setLeftRearPodXYOffsets(double[] leftRearPodXYOffsets) {
        this.leftRearPodXYOffsets = leftRearPodXYOffsets;
    }

    public void setRightRearPodXYOffsets(double[] rightRearPodXYOffsets) {
        this.rightRearPodXYOffsets = rightRearPodXYOffsets;
    }




    public void defaults() {
        leftFrontMotorName = "frontLeftDrive";
        leftFrontServoName = "frontLeftTurnServo";
        leftFrontEncoderName = "frontLeftTurnEncoder";
        rightFrontMotorName = "frontRightDrive";
        rightFrontServoName = "frontRightTurnServo";
        rightFrontEncoderName = "frontRightTurnEncoder";
        leftRearMotorName = "backLeftDrive";
        leftRearServoName = "backLeftTurnServo";
        leftRearEncoderName = "backLeftTurnEncoder";
        rightRearMotorName = "backRightDrive";
        rightRearServoName = "backRightTurnServo";
        rightRearEncoderName = "backRightTurnEncoder";
        leftFrontTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
        rightFrontTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
        leftRearTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
        rightRearTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
        leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        leftFrontServoReversed = false;
        rightFrontServoReversed = false;
        leftRearServoReversed = false;
        rightRearServoReversed = false;
        leftFrontPodAngleOffsetDeg = 0.0;
        rightFrontPodAngleOffsetDeg = 0.0;
        leftRearPodAngleOffsetDeg = 0.0;
        rightRearPodAngleOffsetDeg = 0.0;
        leftFrontPodXYOffsets = new double[] {-1, 1};
        rightFrontPodXYOffsets = new double[] {1, 1};
        leftRearPodXYOffsets = new double[] {-1, -1};
        rightRearPodXYOffsets = new double[] {1, -1};
    }
}
