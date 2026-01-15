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
 * Swerve Drivetrain implementation
 * 
 * @author Kabir Goyal - 365 MOE
 * @author Baron Henderson
 */
public class Swerve extends CustomDrivetrain {
    private final SwerveConstants constants;

    protected Vector lastTranslationalVector = new Vector();
    protected Vector lastHeadingPower = new Vector();
    protected Vector lastCorrectivePower = new Vector();
    protected Vector lastPathingPower = new Vector();
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
     * @param forward  the forward power value, which would typically be
     *                 -gamepad1.left_stick_y in a normal arcade drive setup.
     * @param strafe   the strafe power value, which would typically be
     *                 gamepad1.left_stick_x in a normal arcade drive setup.
     * @param rotation the rotation power value, which would typically be
     *                 gamepad1.right_stick_x in a normal arcade drive setup.
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
            double currentAngle = pods.get(i).getAngle();

            // ask the pod to translate the wheel-space theta into the encoder frame
            double encodedRad = pods.get(i).adjustThetaForEncoder(podVectors[i].getTheta());

            // current and target in radians
            double currentRad = Math.toRadians(currentAngle);
            double targetRad = encodedRad; // already normalized in radians by implementations

            // compute shortest signed error in radians using MathFunctions
            double mag = MathFunctions.getSmallestAngleDifference(currentRad, targetRad);
            double dir = MathFunctions.getTurnDirection(currentRad, targetRad);
            double signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;
            double errorDeg = Math.toDegrees(signedRad);

            avgScaling += Math.abs(Math.cos(Math.toRadians(errorDeg)));
        }

        avgScaling /= pods.size();
        lastAvgScaling = avgScaling;

        for (int podNum = 0; podNum < pods.size(); podNum++) {
            // Normalizing if necessary while preserving relative sizes
            Vector finalVector = podVectors[podNum].times(maxPowerScaling / maxMagnitude);

            // 2*Pi-theta because servos have positive clockwise rotation, while our angles
            // are counterclockwise, and we want to see if motor/servo caching is an issue
            pods.get(podNum).move(finalVector.getTheta(), finalVector.getMagnitude() * avgScaling,
    zeroTrans && zeroRotation && constants.getZeroPowerBehavior() == SwerveConstants.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
        }
    }

    @Override
    public void updateConstants() {
        this.useBrakeModeInTeleOp = constants.getUseBrakeModeInTeleOp();
        this.maxPowerScaling = constants.getMaxPower(); // inherited from Drivetrain, used by CustomDrivetrain
        this.voltageCompensation = constants.getUseVoltageCompensation(); // inherited from Drivetrain
        this.nominalVoltage = constants.getNominalVoltage(); // inherited from Drivetrain
        this.staticFrictionCoefficient = constants.getStaticFrictionCoefficient();
        this.epsilon = constants.getEpsilon();
    }

    @Override
    public void breakFollowing() {
        for (SwervePod pod : pods) {
            pod.move(Math.toRadians(pod.getAngle()), 0, false);
            pod.setToFloat();
        }
    }

    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            for (SwervePod pod : pods) {
                pod.setToBreak();
            }
        }
    }

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

    @Override
    public double xVelocity() {
        return constants.getXVelocity();
    }

    @Override
    public double yVelocity() {
        return constants.getYVelocity();
    }

    @Override
    public void setXVelocity(double xMovement) {
        constants.setXVelocity(xMovement);
    }

    @Override
    public void setYVelocity(double yMovement) {
        constants.setYVelocity(yMovement);
    }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage
                - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    @Override
    public String debugString() {
        StringBuilder sb = new StringBuilder("Swerve {");
        for (int i = 0; i < pods.size(); i++) {
            sb.append("\n pod").append(i).append(" angle = ").append(pods.get(i).getAngle())
                    .append(" debug = ").append(pods.get(i).debugString());
        }
        sb.append("\n forward input=").append(lastForward)
                .append("\n, strafe input=").append(lastStrafe)
                .append("\n, rotation input=").append(lastRotation)
                .append("\n, unrotated translationVector x").append(lastTranslationalVector.getXComponent())
                .append("\n, unrotated translationVector y").append(lastTranslationalVector.getYComponent())
                .append("\n, correctivePower x").append(lastCorrectivePower.getXComponent())
                .append("\n, correctivePower y").append(lastCorrectivePower.getYComponent())
                .append("\n, pathingPower x").append(lastPathingPower.getXComponent())
                .append("\n, pathingPower y").append(lastPathingPower.getYComponent())
                .append("\n, headingPower magnitude").append(lastHeadingPower.getMagnitude())
                .append("\n, headingPower direction").append(lastHeadingPower.getTheta())
                .append("\nrobot heading").append(lastHeading)
                .append("\navg scaling").append(lastAvgScaling)
                .append("\n}");
        return sb.toString();
    }
}