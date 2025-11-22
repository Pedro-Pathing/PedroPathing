package com.pedropathing.ftc.drivetrains;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Handles the rotation and movement of each individual swerve pod
 * @author Kabir Goyal - 365 MOE
 */
public class SwervePod {
    private final AnalogInput turnEncoder; //for rotation of servo
    private final CRServo turnServo;
    private final DcMotor driveMotor;

    private final PIDFController turnPID;

    private final double angleOffsetDeg;
    private final double xOffset;
    private final double yOffset;

    private Telemetry telemetry;
    private final String servoLabel;

    // REV analog reference voltage (0–3.3 V)
    private static final double ANALOG_REF_V = 3.3;


    /**
     * Constructs the Swerve Pod
     * @param servoName Name of the pod's rotation servo
     * @param encoderName Name of the pod's encoder input
     * @param motorName Name of the pod's drive motor
     * @param pidfCoefficients PIDF coefficients for the pod's rotation control
     * @param driveDirection Direction of the drive motor
     * @param servoDirection Direction of the servo
     * @param angleOffsetDeg In degrees, the negative of what the encoder reads when the pod is facing forward
     * @param offsets Array of the pod's x and y offsets from the robot center (units don't matter, just relative size of x and y)
     */
    public SwervePod(HardwareMap hardwareMap, String servoName, String encoderName, String motorName,
                     PIDFCoefficients pidfCoefficients, DcMotorSimple.Direction driveDirection, CRServo.Direction servoDirection,
                     double angleOffsetDeg, double[] offsets) {
        this.turnServo = hardwareMap.get(CRServo.class, servoName);
        this.turnEncoder = hardwareMap.get(AnalogInput.class, encoderName);
        this.driveMotor = hardwareMap.get(DcMotor.class, motorName);

        this.turnPID = new PIDFController(pidfCoefficients);
        this.angleOffsetDeg = angleOffsetDeg;
        this.xOffset = offsets[0];
        this.yOffset = offsets[1];

        this.servoLabel = servoName;

        //I think we want float for pathing?
        setMotorToFloat();
//        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveMotor.setDirection(driveDirection);

        turnServo.setDirection(servoDirection);

//        enableServo();
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


//    public void disableServo() {
//        turnServo.getController().pwmDisable();
//    }

//    public void enableServo() {
//        turnServo.getController().pwmEnable();
//    }

    /**
     * Commands pod to a wheel heading (degrees) with a drive power [0, 1]
     */
    public void move(double targetAngleRad, double drivePower, boolean ignoreServoAngleChanges, double motorCachingThreshold, double servoCachingThreshold) {
        double actualDeg = normalizeNeg180To180(getRawAngleDeg() - angleOffsetDeg);
        //add 90 because servo 0s are facing forward, not to the right
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

        //before pedro if something breaks
        //double turnPower = -clamp(turnPID.calculate(setpointDeg, actualDeg), -1.0, 1.0);

        turnPID.updateError(setpointDeg - actualDeg);
        double turnPower = -clamp(turnPID.run(), -1.0, 1.0);

        if (ignoreServoAngleChanges) {
           turnServo.setPower(0);
        } else if (Math.abs(turnPower - turnServo.getPower()) > servoCachingThreshold) {
            turnServo.setPower(turnPower);
        }




        if (Math.abs(drivePower - driveMotor.getPower()) > motorCachingThreshold)
            driveMotor.setPower(drivePower);
    }

    public double getXOffset() {
        return xOffset;
    }

    public double getYOffset() {
        return yOffset;
    }

    public double getRawAngleDeg() {
        // Map 0–3.3 V -> 0–360°
        return (turnEncoder.getVoltage() / ANALOG_REF_V) * 360.0;
    }

    public double getOffsetAngleDeg() {
        return normalize0To360(getRawAngleDeg() - angleOffsetDeg);
    }

    private static double normalize0To360(double deg) {
        deg = deg % 360.0;
        if (deg < 0)
            deg += 360.0;
        return deg;
    }

    private static double normalizeNeg180To180(double deg) {
        deg = deg % 360.0;
        if (deg < -180)
            deg += 360.0;
        else if (deg > 180) {
            deg -= 360.0;
        }
        return deg;
    }

    /** Smallest signed delta from current to target in [-180, 180]. */
    private static double shortestAngleToTarget(double current, double target) {
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

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(v, hi));
    }

    public String debugString() {
        return servoLabel + "{" +
                "current Angle=" + getRawAngleDeg() +
                ", servo Power=" + turnServo.getPower() +
                ", drive Power=" + driveMotor.getPower() +
                " }";
    }
}