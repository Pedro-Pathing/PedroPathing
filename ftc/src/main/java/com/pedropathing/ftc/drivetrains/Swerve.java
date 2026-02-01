package com.pedropathing.ftc.drivetrains;

import com.pedropathing.drivetrain.CustomDrivetrain;
import com.pedropathing.math.Vector;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.List;

/**
 * Swerve Drivetrain implementation.
 * Angles are in radians and positive rotation is to the left (CCW, top-down).
 *
 * @author Kabir Goyal
 * @author Baron Henderson
 */
public class Swerve extends CustomDrivetrain {
    private final SwerveConstants constants;

    protected double lastHeading = 0;

    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;
    private double epsilon;

    private List<SwervePod> pods;

    private double lastForward = 0;
    private double lastStrafe = 0;
    private double lastRotation = 0;
    private double lastAvgScaling = 0;

    private final VoltageSensor voltageSensor;

    /**
     * @param constants Swerve Contants for your bot
     * @param pods SwervePods, coaxial or differential
     */
    public Swerve(HardwareMap hardwareMap, SwerveConstants constants, SwervePod... pods) {
        this.constants = constants;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        updateConstants();
        this.pods = Arrays.asList(pods);
    }

    /**
     * This method takes in forward, strafe, and rotation values and applies them to
     * the drivetrain.
     *
     * @param forward the forward power value, which would typically be
     *                -gamepad1.left_stick_y in a normal arcade drive setup
     * @param strafe the strafe power value, which would typically be
     *               -gamepad1.left_stick_x in a normal arcade drive setup
     *               because pedro treats left as positive
     * @param rotation the rotation power value, which would typically be
     *                 -gamepad1.right_stick_x in a normal arcade drive setup
     *                 because CCW is positive
     */
    public void arcadeDrive(double forward, double strafe, double rotation) {
        strafe *= -1;

        lastForward = forward;
        lastStrafe = strafe;
        lastRotation = rotation;

        // stores forward and strafe values as the translation vector with max magnitude of 1
        Vector rawTrans = new Vector(Range.clip(Math.hypot(strafe, forward), 0, 1), Math.atan2(forward, strafe));

        boolean zeroTrans = rawTrans.getMagnitude() < epsilon;
        boolean zeroRotation = Math.abs(rotation) < epsilon;

        double rotationScalar = (zeroRotation) ? 0 : rotation;

        Vector[] podVectors = new Vector[pods.size()];

        for (int i = 0; i < pods.size(); i++) {
            SwervePod pod = pods.get(i);

            Vector translationVector = zeroTrans ? new Vector(0, 0) : rawTrans;

            // actually positive rotation scalar because positive turning is to the left
            Vector rotationVector = new Vector(rotationScalar, Math.atan2(pod.getOffset().getX(), -pod.getOffset().getY()));

            // this gets the perpendicular vector for the wheel
            rotationVector.rotateVector(Math.PI / 2);

            podVectors[i] = translationVector.plus(rotationVector);
            if (constants.getZeroPowerBehavior() == SwerveConstants.ZeroPowerBehavior.RESIST_MOVEMENT
                    && zeroTrans && zeroRotation) {
                podVectors[i] = rotationVector;
            }
        }

        // finding if any vector has magnitude > maxPowerScaling
        double maxMagnitude = maxPowerScaling;
        for (Vector podVector : podVectors) {
            if (voltageCompensation) {
                double voltageNormalized = getVoltageNormalized();
                podVector.times(voltageNormalized);
            }
            maxMagnitude = Math.max(maxMagnitude, podVector.getMagnitude());
        }

        // Find the avg scaling constant (avg of cos(angle error))
        double avgScaling = 0;

        for (int i = 0; i < pods.size(); i++) {
            double currentRad = pods.get(i).getAngle();

            // ask the pod to translate the wheel-space theta into the encoder frame
            double targetRad = pods.get(i).adjustThetaForEncoder(podVectors[i].getTheta());

            // compute shortest signed error in radians using MathFunctions
            double mag = MathFunctions.getSmallestAngleDifference(currentRad, targetRad);
            double dir = MathFunctions.getTurnDirection(currentRad, targetRad);
            double errorRad = (mag == Math.PI) ? -Math.PI : mag * dir;

            avgScaling += Math.abs(Math.cos(errorRad));
        }

        avgScaling /= pods.size();
        lastAvgScaling = avgScaling;

        for (int podNum = 0; podNum < pods.size(); podNum++) {
            // Normalizing if necessary while preserving relative sizes
            Vector finalVector = podVectors[podNum].times(maxPowerScaling / maxMagnitude);

            pods.get(podNum).move(finalVector.getTheta(), finalVector.getMagnitude() * avgScaling,
    zeroTrans && zeroRotation && constants.getZeroPowerBehavior() == SwerveConstants.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
        }
    }

    /**
     * Updates cached values from the constants object.
     */
    @Override
    public void updateConstants() {
        this.useBrakeModeInTeleOp = constants.getUseBrakeModeInTeleOp();
        this.maxPowerScaling = constants.getMaxPower(); // inherited from Drivetrain, used by CustomDrivetrain
        this.voltageCompensation = constants.getUseVoltageCompensation(); // inherited from Drivetrain
        this.nominalVoltage = constants.getNominalVoltage(); // inherited from Drivetrain
        this.staticFrictionCoefficient = constants.getStaticFrictionCoefficient();
        this.epsilon = constants.getEpsilon();
    }

    /**
     * Stops following and holds pod angles while floating drive motors.
     */
    @Override
    public void breakFollowing() {
        for (SwervePod pod : pods) {
            pod.move(pod.getAngle(), 0, true);
            pod.setToFloat();
        }
    }

    /**
     * Starts teleop drive with the configured brake mode.
     */
    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            for (SwervePod pod : pods) {
                pod.setToBreak();
            }
        }
    }

    /**
     * @param brakeMode set to true to enable brake mode in teleop
     */
    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            for (SwervePod pod : pods) {
                pod.setToBreak();
            }
        } else {
            for (SwervePod pod : pods) {
                pod.setToFloat();
            }
        }
    }

    /**
     * @return maximum x velocity
     */
    @Override
    public double xVelocity() {
        return constants.getXVelocity();
    }

    /**
     * @return maximum y velocity
     */
    @Override
    public double yVelocity() {
        return constants.getYVelocity();
    }

    /**
     * @param xMovement maximum x velocity
     */
    @Override
    public void setXVelocity(double xMovement) {
        constants.setXVelocity(xMovement);
    }

    /**
     * @param yMovement maximum y velocity
     */
    @Override
    public void setYVelocity(double yMovement) {
        constants.setYVelocity(yMovement);
    }

    /**
     * @return static friction coefficient used for voltage compensation
     */
    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    /**
     * @return current battery voltage
     */
    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    /**
     * @return normalized voltage for voltage compensation
     */
    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage
                - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    /**
     * @return debug string for drivetrain state
     */
    @Override
    public String debugString() {
        StringBuilder sb = new StringBuilder("Swerve {");
        for (int i = 0; i < pods.size(); i++) {
            sb.append("\npod").append(i)
                    .append(": ").append(pods.get(i).debugString());
        }
        sb.append("\n\nforward input=").append(lastForward)
                .append("\nstrafe input=").append(lastStrafe)
                .append("\nrotation input=").append(lastRotation)
                .append("\nrobot heading").append(lastHeading)
                .append("\navg scaling").append(lastAvgScaling)
                .append("\n}");
        return sb.toString();
    }
}
