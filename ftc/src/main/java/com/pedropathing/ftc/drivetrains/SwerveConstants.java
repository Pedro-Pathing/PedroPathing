package com.pedropathing.ftc.drivetrains;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Constants for swerve drive configuration
 * @author Kabir Goyal - 365 MOE
 */
public class SwerveConstants {

    public double xVelocity = 80.0; //TODO: Change based on your robot
    public double yVelocity = 80.0; //TODO: Change based on your robot

    public boolean useBrakeModeInTeleOp = false;
    public double maxPower = 1.0;
    public double motorCachingThreshold = 0.01;
    public double servoCachingThreshold = 0.01;
    public boolean useVoltageCompensation = false;
    public double nominalVoltage = 12.0;
    public double staticFrictionCoefficient = 0.1;

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
    public CRServo.Direction leftFrontServoDirection = CRServo.Direction.FORWARD;
    public CRServo.Direction rightFrontServoDirection = CRServo.Direction.FORWARD;
    public CRServo.Direction leftRearServoDirection = CRServo.Direction.FORWARD;
    public CRServo.Direction rightRearServoDirection = CRServo.Direction.FORWARD;

    //TODO: These are the reported angle of each pod when facing forward, in degrees
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

    public double leftFrontReferenceVoltage = 3.3;
    public double rightFrontReferenceVoltage = 3.3;
    public double leftRearReferenceVoltage = 3.3;
    public double rightRearReferenceVoltage = 3.3;

    //use to reverse your encoder if positive is counterclockwise
    public boolean leftFrontEncoderReversed = false;
    public boolean rightFrontEncoderReversed = false;
    public boolean leftRearEncoderReversed = false;
    public boolean rightRearEncoderReversed = false;


    public SwerveConstants() {
        defaults();
    }


    /**
     * @param velocity the max speed in ANY direction because swerve can do that :)
     * @return
     */
    public SwerveConstants velocity(double velocity) {
        this.xVelocity = velocity;
        this.yVelocity = velocity;
        return this;
    }

    public SwerveConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public SwerveConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public SwerveConstants useBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
        return this;
    }

    public SwerveConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public SwerveConstants motorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
        return this;
    }

    public SwerveConstants servoCachingThreshold(double servoCachingThreshold) {
        this.servoCachingThreshold = servoCachingThreshold;
        return this;
    }

    public SwerveConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public SwerveConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public SwerveConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
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

    public SwerveConstants leftFrontServoDirection(CRServo.Direction leftFrontServoDirection) {
        this.leftFrontServoDirection = leftFrontServoDirection;
        return this;
    }

    public SwerveConstants rightFrontServoDirection(CRServo.Direction rightFrontServoDirection) {
        this.rightFrontServoDirection = rightFrontServoDirection;
        return this;
    }

    public SwerveConstants leftRearServoDirection(CRServo.Direction leftRearServoDirection) {
        this.leftRearServoDirection = leftRearServoDirection;
        return this;
    }

    public SwerveConstants rightRearServoDirection(CRServo.Direction rightRearServoDirection) {
        this.rightRearServoDirection = rightRearServoDirection;
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


    public SwerveConstants leftFrontReferenceVoltage(double leftFrontReferenceVoltage) {
        this.leftFrontReferenceVoltage = leftFrontReferenceVoltage;
        return this;
    }
    public SwerveConstants rightFrontReferenceVoltage(double rightFrontReferenceVoltage) {
        this.rightFrontReferenceVoltage = rightFrontReferenceVoltage;
        return this;
    }
    public SwerveConstants leftRearReferenceVoltage(double leftRearReferenceVoltage) {
        this.leftRearReferenceVoltage = leftRearReferenceVoltage;
        return this;
    }
    public SwerveConstants rightRearReferenceVoltage(double rightRearReferenceVoltage) {
        this.rightRearReferenceVoltage = rightRearReferenceVoltage;
        return this;
    }

    public SwerveConstants leftFrontEncoderReversed(boolean leftFrontEncoderReversed) {
        this.leftFrontEncoderReversed = leftFrontEncoderReversed;
        return this;
    }

    public SwerveConstants rightFrontEncoderReversed(boolean rightFrontEncoderReversed) {
        this.rightFrontEncoderReversed = rightFrontEncoderReversed;
        return this;
    }

    public SwerveConstants leftRearEncoderReversed(boolean leftRearEncoderReversed) {
        this.leftRearEncoderReversed = leftRearEncoderReversed;
        return this;
    }
    public SwerveConstants rightRearEncoderReversed(boolean rightRearEncoderReversed) {
        this.rightRearEncoderReversed = rightRearEncoderReversed;
        return this;
    }
    public double getVelocity() {
        return xVelocity; //almost certainly for swerve xVel = yVel
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public boolean getUseBrakeModeInTeleOp() {
        return useBrakeModeInTeleOp;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public double getMotorCachingThreshold() {
        return motorCachingThreshold;
    }

    public double getServoCachingThreshold() {
        return servoCachingThreshold;
    }

    public boolean getUseVoltageCompensation() {
        return useVoltageCompensation;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
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

    public CRServo.Direction getLeftFrontServoDirection() {
        return leftFrontServoDirection;
    }

    public CRServo.Direction getRightFrontServoDirection() {
        return rightFrontServoDirection;
    }

    public CRServo.Direction getLeftRearServoDirection() {
        return leftRearServoDirection;
    }

    public CRServo.Direction getRightRearServoDirection() {
        return rightRearServoDirection;
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

    public double getLeftFrontReferenceVoltage() {
        return leftFrontReferenceVoltage;
    }
    public double getRightFrontReferenceVoltage() {
        return rightFrontReferenceVoltage;
    }
    public double getLeftRearReferenceVoltage() {
        return leftRearReferenceVoltage;
    }
    public double getRightRearReferenceVoltage() {
        return rightRearReferenceVoltage;
    }

    public boolean getLeftFrontEncoderReversed() {
        return leftFrontEncoderReversed;
    }

    public boolean getRightFrontEncoderReversed() {
        return rightFrontEncoderReversed;
    }
    public boolean getLeftRearEncoderReversed() {
        return leftRearEncoderReversed;
    }
    public boolean getRightRearEncoderReversed() {
        return rightRearEncoderReversed;
    }
    public void setVelocity(double velocity) {
        this.xVelocity = velocity;
        this.yVelocity = velocity;
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public void setUseBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public void setServoCachingThreshold(double servoCachingThreshold) {
        this.servoCachingThreshold = servoCachingThreshold;
    }

    public void setUseVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    public void setStaticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
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

    public void setLeftFrontServoDirection(CRServo.Direction leftFrontServoDirection) {
        this.leftFrontServoDirection = leftFrontServoDirection;
    }

    public void setRightFrontServoDirection(CRServo.Direction rightFrontServoDirection) {
        this.rightFrontServoDirection = rightFrontServoDirection;
    }

    public void setLeftRearServoDirection(CRServo.Direction leftRearServoDirection) {
        this.leftRearServoDirection = leftRearServoDirection;
    }

    public void setRightRearServoDirection(CRServo.Direction rightRearServoDirection) {
        this.rightRearServoDirection = rightRearServoDirection;
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

    public void setLeftFrontReferenceVoltage(double leftFrontReferenceVoltage) {
        this.leftFrontReferenceVoltage = leftFrontReferenceVoltage;
    }
    public void setRightFrontReferenceVoltage(double rightFrontReferenceVoltage) {
        this.rightFrontReferenceVoltage = rightFrontReferenceVoltage;
    }
    public void setLeftRearReferenceVoltage(double leftRearReferenceVoltage) {
        this.leftRearReferenceVoltage = leftRearReferenceVoltage;
    }
    public void setRightRearReferenceVoltage(double rightRearReferenceVoltage) {
        this.rightRearReferenceVoltage = rightRearReferenceVoltage;
    }

    public void setLeftFrontEncoderReversed(boolean leftFrontEncoderReversed) {
        this.leftFrontEncoderReversed = leftFrontEncoderReversed;
    }
    public void setRightFrontEncoderReversed(boolean rightFrontEncoderReversed) {
        this.rightFrontEncoderReversed = rightFrontEncoderReversed;
    }
    public void setLeftRearEncoderReversed(boolean leftRearEncoderReversed) {
        this.leftRearEncoderReversed = leftRearEncoderReversed;
    }
    public void setRightRearEncoderReversed(boolean rightRearEncoderReversed) {
        this.rightRearEncoderReversed = rightRearEncoderReversed;
    }




    public void defaults() {
        xVelocity = 80.0;
        yVelocity = 80.0;
        useBrakeModeInTeleOp = false;
        maxPower = 1.0;
        motorCachingThreshold = 0.01;
        servoCachingThreshold = 0.01;
        useVoltageCompensation = false;
        nominalVoltage = 12.0;
        staticFrictionCoefficient = 0.1;
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
        leftFrontServoDirection = CRServo.Direction.FORWARD;
        rightFrontServoDirection = CRServo.Direction.FORWARD;
        leftRearServoDirection = CRServo.Direction.FORWARD;
        rightRearServoDirection = CRServo.Direction.FORWARD;
        leftFrontPodAngleOffsetDeg = 0.0;
        rightFrontPodAngleOffsetDeg = 0.0;
        leftRearPodAngleOffsetDeg = 0.0;
        rightRearPodAngleOffsetDeg = 0.0;
        leftFrontPodXYOffsets = new double[] {-1, 1};
        rightFrontPodXYOffsets = new double[] {1, 1};
        leftRearPodXYOffsets = new double[] {-1, -1};
        rightRearPodXYOffsets = new double[] {1, -1};
        leftFrontReferenceVoltage = 3.3;
        rightFrontReferenceVoltage = 3.3;
        leftRearReferenceVoltage = 3.3;
        rightRearReferenceVoltage = 3.3;
        leftFrontEncoderReversed = false;
        rightFrontEncoderReversed = false;
        leftRearEncoderReversed = false;
        rightRearEncoderReversed = false;
    }
}