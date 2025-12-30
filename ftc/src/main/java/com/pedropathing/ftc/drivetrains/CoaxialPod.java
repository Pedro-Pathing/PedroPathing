package com.pedropathing.ftc.drivetrains;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.*;

/**
 * CoaxialPod is a hardware-backed implementation of the core `SwervePod` interface.
 * It owns the drive motor, continuous rotation servo (turn), analog encoder and the
 * PIDF controller used to control pod rotation.
 * @author Baron Henderson
 */
public class CoaxialPod implements SwervePod {
  private final AnalogInput turnEncoder; // for rotation of servo
  private final CRServo turnServo;
  private final DcMotorEx driveMotor;

  private final PIDFController turnPID;
  private final Pose offset;

  private final double angleOffsetDeg;

  private final String servoLabel;

  // analog encoder voltage range (min -> max), e.g. 0.0 -> 3.3 V
  private final double analogMinVoltage;
  private final double analogMaxVoltage;

  private final boolean encoderReversed;

  private double motorCachingThreshold = 0.01;
  private double servoCachingThreshold = 0.01;
  private double feedForward = 0.0;

  public CoaxialPod(HardwareMap hardwareMap, String motorName, String servoName, String turnEncoderName,
                    PIDFCoefficients turnPIDFCoefficients, DcMotorSimple.Direction driveDirection,
                    CRServo.Direction servoDirection, double angleOffsetDeg, Pose podOffset,
                    double analogMinVoltage, double analogMaxVoltage, boolean encoderReversed, double drivePIDFFeedForward) {

      this.driveMotor = hardwareMap.get(DcMotorEx.class, motorName); 
      this.turnServo = hardwareMap.get(CRServo.class, servoName);
      this.turnEncoder = hardwareMap.get(AnalogInput.class, turnEncoderName);

    this.servoLabel = turnServo.getConnectionInfo(); // Best guess for label without explicit name

    this.turnPID = new PIDFController(turnPIDFCoefficients);
    this.angleOffsetDeg = angleOffsetDeg;

    setMotorToFloat();

    driveMotor.setDirection(driveDirection);
    turnServo.setDirection(servoDirection);

    this.analogMinVoltage = analogMinVoltage;
    this.analogMaxVoltage = analogMaxVoltage;
    this.encoderReversed = encoderReversed;
    this.feedForward = drivePIDFFeedForward;
    
    this.offset = podOffset;
  }

  @Override
  public Pose getOffset() { return offset; }

  @Override
  public double getAngle() {
    return getAngleAfterOffsetDeg();
  }

  public void setServoPower(double power) {
    turnServo.setPower(power);
  }

  public void setMotorPower(double power) {
    driveMotor.setPower(power);
  }

  @Override
  public void setToFloat() { setMotorToFloat(); }

  @Override
  public void setToBreak() { setMotorToBreak(); }

  public void setMotorToFloat() {
    driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
  }

  public void setMotorToBreak() {
    driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  public boolean isEncoderReversed() {
    return encoderReversed;
  }

  @Override
  public double adjustThetaForEncoder(double wheelTheta) {
    // wheelTheta is in radians. If encoder is reversed, use wheelTheta directly; otherwise invert.
    double t = encoderReversed ? wheelTheta : (2 * Math.PI - wheelTheta);
    // servo zero offset: +90 degrees -> +pi/2 radians
    t += Math.PI / 2.0;
    return MathFunctions.normalizeAngle(t);
  }

  /**
   * Commands pod to a wheel heading (radians) with a drive power [0, 1]
   * This implementation pulls caching thresholds and feedforward from internal state.
   */
  @Override
  public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
    // Convert hardware angle (degrees) to radians and normalize
    double actualRad = Math.toRadians(getAngleAfterOffsetDeg());
    actualRad = MathFunctions.normalizeAngle(actualRad);

    // Account for encoder orientation and servo zero offset (+90deg)
    double desiredRad = encoderReversed ? targetAngleRad : (2 * Math.PI - targetAngleRad);
    desiredRad += Math.PI / 2.0;
    desiredRad = MathFunctions.normalizeAngle(desiredRad);

    // Shortest-path error in radians (signed)
    double mag = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
    double dir = MathFunctions.getTurnDirection(actualRad, desiredRad);
    double signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;

    // Convert to degrees for PID, preserving signedness
    double errorDeg = Math.toDegrees(signedRad);

    // Minimize rotation: flip + invert drive if > 90°
    if (Math.abs(errorDeg) > 90.0) {
      // add 180 degrees (pi radians)
      desiredRad = MathFunctions.normalizeAngle(desiredRad + Math.PI);
      drivePower = -drivePower;

      // recompute signed error
      mag = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
      dir = MathFunctions.getTurnDirection(actualRad, desiredRad);
      signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;
      errorDeg = Math.toDegrees(signedRad);
    }

    // Setpoint close to current so PID follows shortest path
    double actualDeg = Math.toDegrees(actualRad);
    double setpointDeg = actualDeg + errorDeg;

    turnPID.updateError(setpointDeg - actualDeg);
    double turnPower = -MathFunctions.clamp(turnPID.run(), -1.0, 1.0);

    // Add feedforward if we have non-negligible error
    if (Math.abs(errorDeg) > 0.02) {
      turnPower += feedForward * Math.signum(turnPower);
    }

    if (ignoreAngleChanges) {
      turnPower = 0;
    }

    if (Math.abs(turnPower - turnServo.getPower()) > servoCachingThreshold)
      turnServo.setPower(turnPower);

    if (Math.abs(drivePower - driveMotor.getPower()) > motorCachingThreshold)
      driveMotor.setPower(drivePower);
  }

  public double getAngleAfterOffsetDeg() {
    return getRawAngleDeg() - angleOffsetDeg;
  }

  public double getRawAngleDeg() {
    double v = turnEncoder.getVoltage();
    double range = analogMaxVoltage - analogMinVoltage;
    if (range == 0) return 0;
    double normalized = (v - analogMinVoltage) / range;
    normalized = MathFunctions.clamp(normalized, 0, 1);
    return normalized * 360.0;
  }

  public double getOffsetAngleDeg() {
    double deg = getRawAngleDeg() - angleOffsetDeg;
    return Math.toDegrees(MathFunctions.normalizeAngle(Math.toRadians(deg)));
  }

  public void setMotorCachingThreshold(double motorCachingThreshold) {
    this.motorCachingThreshold = motorCachingThreshold;
  }

  public void setServoCachingThreshold(double servoCachingThreshold) {
    this.servoCachingThreshold = servoCachingThreshold;
  }

  public void setFeedForward(double feedForward) {
    this.feedForward = feedForward;
  }

  @Override
  public String debugString() {
    return servoLabel + "{" + "current Angle=" + getRawAngleDeg() + ", servo Power="
        + turnServo.getPower() + ", drive Power=" + driveMotor.getPower() + " }";
  }
}
