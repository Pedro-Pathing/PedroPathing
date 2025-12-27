package com.pedropathing.ftc.drivetrains;
import com.pedropathing.drivetrain.CustomDrivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Swerve drivetrain implementation
 * 
 * @author Kabir Goyal - 365 MOE
 */
public class Swerve extends CustomDrivetrain {
    private final SwerveConstants constants;

    protected Vector lastTranslationalVector = new Vector();
    protected Vector lastHeadingPower = new Vector();
    protected Vector lastCorrectivePower = new Vector();
    protected Vector lastPathingPower = new Vector();
    protected double lastHeading = 0;

    private boolean useBrakeModeInTeleOp; // implemented
    private double motorCachingThreshold;
    private double servoCachingThreshold;
    private double staticFrictionCoefficient;
    private double drivePIDFFeedForward;
    private double epsilon;

    private final SwervePod leftFrontPod;
    private final SwervePod rightFrontPod;
    private final SwervePod leftRearPod;
    private final SwervePod rightRearPod;

    private double lastForward = 0;
    private double lastStrafe = 0;
    private double lastRotation = 0;
    private double lastAvgScaling = 0;

    private final VoltageSensor voltageSensor;

    private final SwervePod[] pods;

    private final HardwareMap hardwareMap;

    public Swerve(HardwareMap hardwareMap, SwerveConstants constants) {
        this.hardwareMap = hardwareMap;
        this.constants = constants;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        pods = new SwervePod[4];
        updateConstants();

        leftFrontPod = new SwervePod(
                hardwareMap.get(DcMotorEx.class, constants.getLeftFrontMotorName()),
                hardwareMap.get(CRServo.class, constants.getLeftFrontServoName()),
                hardwareMap.get(AnalogInput.class, constants.getLeftFrontEncoderName()),
                constants.getLeftFrontTurnPID(),
                constants.getLeftFrontMotorDirection(), constants.getLeftFrontServoDirection(),
                constants.getLeftFrontPodAngleOffsetDeg(), constants.getLeftFrontPodXYOffsets(),
                constants.getLeftFrontReferenceVoltage(), constants.getLeftFrontEncoderReversed());

        rightFrontPod = new SwervePod(
                hardwareMap.get(DcMotorEx.class, constants.getRightFrontMotorName()),
                hardwareMap.get(CRServo.class, constants.getRightFrontServoName()),
                hardwareMap.get(AnalogInput.class, constants.getRightFrontEncoderName()),
                constants.getRightFrontTurnPID(),
                constants.getRightFrontMotorDirection(), constants.getRightFrontServoDirection(),
                constants.getRightFrontPodAngleOffsetDeg(), constants.getRightFrontPodXYOffsets(),
                constants.getRightFrontReferenceVoltage(), constants.getRightFrontEncoderReversed());

        leftRearPod = new SwervePod(
                hardwareMap.get(DcMotorEx.class, constants.getLeftRearMotorName()),
                hardwareMap.get(CRServo.class, constants.getLeftRearServoName()),
                hardwareMap.get(AnalogInput.class, constants.getLeftRearEncoderName()),
                constants.getLeftRearTurnPID(),
                constants.getLeftRearMotorDirection(), constants.getLeftRearServoDirection(),
                constants.getLeftRearPodAngleOffsetDeg(), constants.getLeftRearPodXYOffsets(),
                constants.getLeftRearReferenceVoltage(), constants.getLeftRearEncoderReversed());

        rightRearPod = new SwervePod(
                hardwareMap.get(DcMotorEx.class, constants.getRightRearMotorName()),
                hardwareMap.get(CRServo.class, constants.getRightRearServoName()),
                hardwareMap.get(AnalogInput.class, constants.getRightRearEncoderName()),
                constants.getRightRearTurnPID(),
                constants.getRightRearMotorDirection(), constants.getRightRearServoDirection(),
                constants.getRightRearPodAngleOffsetDeg(), constants.getRightRearPodXYOffsets(),
                constants.getRightRearReferenceVoltage(), constants.getRightRearEncoderReversed());

        pods[0] = leftFrontPod;
        pods[1] = rightFrontPod;
        pods[2] = leftRearPod;
        pods[3] = rightRearPod;
    }

    /**
     * This method takes in forward, strafe, and rotation values and applies them to
     * the drivetrain.
     * Intended to work exactly like an arcade drive would in a typical TeleOp, this
     * method can be a copy pasted from
     * a robot-centric arcade drive implementation.
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

        // stores forward and strafe values as the translation vector with max magnitude
        // of 1
        Vector rawTrans = new Vector(Range.clip(Math.hypot(strafe, forward), 0, 1), Math.atan2(forward, strafe));

        boolean ignoreTrans = rawTrans.getMagnitude() < epsilon;
        boolean ignoreRotation = Math.abs(rotation) < epsilon;

        double rotationScalar = (ignoreRotation) ? 0 : rotation;

        Vector[] podVectors = new Vector[pods.length];

        for (int i = 0; i < pods.length; i++) {
            SwervePod pod = pods[i];

            Vector translationVector = ignoreTrans ? new Vector(0, 0) : rawTrans;

            // actually positive rotation scalar because positive turning is to the left
            Vector rotationVector = new Vector(rotationScalar, Math.atan2(pod.getYOffset(), pod.getXOffset()));

            // this gets the perpendicular vector for the wheel
            rotationVector.rotateVector(Math.PI / 2);

            podVectors[i] = translationVector.plus(rotationVector);
        }

        // finding if any vector has magnitude > maxPowerScaling
        double maxMagnitude = maxPowerScaling;
        for (Vector podVector : podVectors) {
            // voltage compensation impl copied straight from mecanum basically
            if (voltageCompensation) {
                double voltageNormalized = getVoltageNormalized();
                podVector.times(voltageNormalized);
            }
            maxMagnitude = Math.max(maxMagnitude, podVector.getMagnitude());
        }

        // Find the avg scaling constant (avg of cos(angle error))
        double avgScaling = 0;

        for (int i = 0; i < pods.length; i++) {
            double currentAngle = pods[i].getAngleAfterOffsetDeg();
            double target = (pods[i].isEncoderReversed()) ? podVectors[i].getTheta()
                    : 2 * Math.PI - podVectors[i].getTheta();
            target = SwervePod.normalizeNeg180To180(Math.toDegrees(target) + 90);

            double error = SwervePod.shortestAngleToTarget(currentAngle, target);

            avgScaling += Math.abs(Math.cos(Math.toRadians(error)));
        }

        avgScaling /= 4;
        lastAvgScaling = avgScaling;

        for (int podNum = 0; podNum < pods.length; podNum++) {
            // Normalizing if necessary while preserving relative sizes
            Vector finalVector = podVectors[podNum].times(maxPowerScaling / maxMagnitude);

            // 2*Pi-theta because servos have positive clockwise rotation, while our angles
            // are counterclockwise, and we want to see if motor/servo caching is an issue
            pods[podNum].move(finalVector.getTheta(), finalVector.getMagnitude() * avgScaling,
                    ignoreTrans && ignoreRotation, motorCachingThreshold, servoCachingThreshold, drivePIDFFeedForward);
        }
    }

    @Override
    public void updateConstants() {
        this.useBrakeModeInTeleOp = constants.getUseBrakeModeInTeleOp();
        this.maxPowerScaling = constants.getMaxPower(); // inherited from Drivetrain, used by CustomDrivetrain
        this.motorCachingThreshold = constants.getMotorCachingThreshold();
        this.servoCachingThreshold = constants.getServoCachingThreshold();
        this.voltageCompensation = constants.getUseVoltageCompensation(); // inherited from Drivetrain
        this.nominalVoltage = constants.getNominalVoltage(); // inherited from Drivetrain
        this.staticFrictionCoefficient = constants.getStaticFrictionCoefficient();
        this.drivePIDFFeedForward = constants.getDrivePIDFFeedForward();
        this.epsilon = constants.getEpsilon();
    }

    @Override
    public void breakFollowing() {
        for (SwervePod pod : pods) {
            pod.setMotorPower(0);
            pod.setMotorToFloat();
        }
    }

    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            for (SwervePod pod : pods) {
                pod.setMotorToBreak();
            }
        }
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            for (SwervePod pod : pods) {
                pod.setMotorToBreak();
            }
        } else {
            for (SwervePod pod : pods) {
                pod.setMotorToFloat();
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
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient))
                / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    @Override
    public String debugString() {
        return "Swerve{" +
                "\n leftFrontPodAngle = " + leftFrontPod.getAngleAfterOffsetDeg() +
                "\n rightFrontPodAngle = " + rightFrontPod.getAngleAfterOffsetDeg() +
                "\n leftRearPodAngle = " + leftRearPod.getAngleAfterOffsetDeg() +
                "\n rightRearPodAngle = " + rightRearPod.getAngleAfterOffsetDeg() +
                "\nforward input=" + lastForward +
                "\n, strafe input=" + lastStrafe +
                "\n, rotation input=" + lastRotation +
                "\n, unrotated translationVector x" + lastTranslationalVector.getXComponent() +
                "\n, unrotated translationVector y" + lastTranslationalVector.getYComponent() +
                "\n, correctivePower x" + lastCorrectivePower.getXComponent() +
                "\n, correctivePower y" + lastCorrectivePower.getYComponent() +
                "\n, pathingPower x" + lastPathingPower.getXComponent() +
                "\n, pathingPower y" + lastPathingPower.getYComponent() +
                "\n, headingPower magnitude" + lastHeadingPower.getMagnitude() +
                "\n, headingPower direction" + lastHeadingPower.getTheta() +
                "\nrobot heading" + lastHeading +
                "\navg scaling" + lastAvgScaling +
                "\n, leftFront=" + leftFrontPod.debugString() +
                "\n, rightFront=" + rightFrontPod.debugString() +
                "\n, leftRear=" + leftRearPod.debugString() +
                "\n, rightRear=" + rightRearPod.debugString() +
                "\n}";
    }
}