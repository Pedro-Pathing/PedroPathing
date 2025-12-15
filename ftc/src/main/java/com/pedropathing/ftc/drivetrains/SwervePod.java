package com.pedropathing.ftc.drivetrains;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Handles the rotation and movement of each individual swerve pod
 * 
 * @author Kabir Goyal - 365 MOE
 */
public class SwervePod {
  private final AnalogInput turnEncoder; // for rotation of servo
  private final CRServo turnServo;
  private final DcMotorEx driveMotor;

  private final PIDFController turnPID;

  private final double angleOffsetDeg;
  private final double xOffset;
  private final double yOffset;

  private final String servoLabel;

  // REV analog reference voltage (0–3.3 V)
  private final double analogReferenceVoltage;

  private final boolean encoderReversed;

  /**
   * Constructs the Swerve Pod
   * 
   * @param turnServo        The pod's rotation servo
   * @param turnEncoder      The pod's encoder input
   * @param driveMotor       The pod's drive motor
   * @param pidfCoefficients PIDF coefficients for the pod's rotation control
   * @param driveDirection   Direction of the drive motor
   * @param servoDirection   Direction of the servo
   * @param angleOffsetDeg   In degrees, what the encoder reads
   *                         when the pod is facing forward
   * @param offsets          Array of the pod's x and y offsets from the robot
   *                         center (units don't
   *                         matter, just relative size of x and y)
   * @param referenceVoltage Reference voltage for the analog encoder
   * @param encoderReversed  Whether the encoder is reversed
   */
  public SwervePod(DcMotorEx driveMotor, CRServo turnServo, AnalogInput turnEncoder,
      PIDFCoefficients pidfCoefficients, DcMotorSimple.Direction driveDirection,
      CRServo.Direction servoDirection, double angleOffsetDeg, double[] offsets,
      double referenceVoltage, boolean encoderReversed) {
    this.turnServo = turnServo;
    this.turnEncoder = turnEncoder;
    this.driveMotor = driveMotor;

    this.servoLabel = turnServo.getConnectionInfo(); // Best guess for label without explicit name

    this.turnPID = new PIDFController(pidfCoefficients);
    this.angleOffsetDeg = angleOffsetDeg;
    this.xOffset = offsets[0];
    this.yOffset = offsets[1];

    setMotorToFloat();

    driveMotor.setDirection(driveDirection);
    turnServo.setDirection(servoDirection);

    this.analogReferenceVoltage = referenceVoltage;
    this.encoderReversed = encoderReversed;
  }

  public void setServoPower(double power) {
    turnServo.setPower(power);
  }

  public void setMotorPower(double power) {
    driveMotor.setPower(power);
  }

  public void setMotorToFloat() {
    driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
  }

  public void setMotorToBreak() {
    driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  public boolean isEncoderReversed() {
    return encoderReversed;
  }

  /**
   * Commands pod to a wheel heading (degrees) with a drive power [0, 1]
   */
  public void move(double targetAngleRad, double drivePower, boolean ignoreServoAngleChanges,
      double motorCachingThreshold, double servoCachingThreshold, double feedForward) {
    double actualDeg = normalizeNeg180To180(getAngleAfterOffsetDeg());
    // add 90 because servo 0s are facing forward, not to the right
    targetAngleRad = (encoderReversed) ? targetAngleRad : 2 * Math.PI - targetAngleRad;
    double desiredDeg = normalizeNeg180To180(Math.toDegrees(targetAngleRad) + 90);

    // Shortest-path error in [-180, 180]
    double error = shortestAngleToTarget(actualDeg, desiredDeg);

    // Minimize rotation: flip + invert drive if > 90°
    if (Math.abs(error) > 90.0) {
      desiredDeg = normalizeNeg180To180(desiredDeg + 180.0);
      drivePower = -drivePower;
      error = shortestAngleToTarget(actualDeg, desiredDeg);
    }

    // Setpoint close to current so PID follows shortest path
    double setpointDeg = actualDeg + error;

    turnPID.updateError(setpointDeg - actualDeg);
    double turnPower = -MathFunctions.clamp(turnPID.run(), -1.0, 1.0);

    // Add feedforward if error is small (trying to hold position) or large?
    // Usually FF is static friction to get moving.
    // If we have an error, add FF in direction of error.
    if (Math.abs(error) > 0.02) { // Small deadband for FF usage
      turnPower += feedForward * Math.signum(turnPower);
    }

    if (ignoreServoAngleChanges) {
      turnPower = 0;
    }

    if (Math.abs(turnPower - turnServo.getPower()) > servoCachingThreshold)
      turnServo.setPower(turnPower);

    if (Math.abs(drivePower - driveMotor.getPower()) > motorCachingThreshold)
      driveMotor.setPower(drivePower);
  }

  public double getXOffset() {
    return xOffset;
  }

  public double getYOffset() {
    return yOffset;
  }

  public double getAngleAfterOffsetDeg() {
    return getRawAngleDeg() - angleOffsetDeg;
  }

  public double getRawAngleDeg() {
    return (turnEncoder.getVoltage() / analogReferenceVoltage) * 360.0;
  }

  public double getOffsetAngleDeg() {
    return normalize0To360(getRawAngleDeg() - angleOffsetDeg);
  }

  public static double normalize0To360(double deg) {
    deg = deg % 360.0;
    if (deg < 0)
      deg += 360.0;
    return deg;
  }

  public static double normalizeNeg180To180(double deg) {
    deg = deg % 360.0;
    if (deg < -180)
      deg += 360.0;
    else if (deg > 180) {
      deg -= 360.0;
    }
    return deg;
  }

  /** Smallest signed delta from current to target in [-180, 180]. */
  public static double shortestAngleToTarget(double current, double target) {
    current = normalize0To360(current);
    target = normalize0To360(target);

    double delta = target - current;
    if (delta > 180)
      delta -= 360;
    else if (delta <= -180)
      delta += 360;

    if (Math.abs(delta) == 180)
      return -180;
    return delta;
  }

  public String debugString() {
    return servoLabel + "{" + "current Angle=" + getRawAngleDeg() + ", servo Power="
        + turnServo.getPower() + ", drive Power=" + driveMotor.getPower() + " }";
  }
}