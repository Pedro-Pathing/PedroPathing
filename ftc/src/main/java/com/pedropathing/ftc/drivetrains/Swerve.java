package com.pedropathing.ftc.drivetrains;

import com.pedropathing.drivetrain.CustomDrivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;


/**
 * Swerve drivetrain implementation
 * @author Kabir Goyal - 365 MOE
 */
public class Swerve extends CustomDrivetrain {
    public SwerveConstants constants;

    private boolean useBrakeModeInTeleOp; //implemented
    private double motorCachingThreshold;
    private double servoCachingThreshold;
    private double staticFrictionCoefficient;

    private SwervePod leftFrontPod;
    private SwervePod rightFrontPod;
    private SwervePod leftRearPod;
    private SwervePod rightRearPod;

    private final VoltageSensor voltageSensor;

    private final SwervePod[] pods;

    private final HardwareMap hardwareMap;

    public Swerve(HardwareMap hardwareMap, SwerveConstants constants)  {
        this.hardwareMap = hardwareMap;
        this.constants = constants;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        pods = new SwervePod[4];
        updateConstants();
    }

    @Override
    public void arcadeDrive(double forward, double strafe, double rotation) {

        //stores forward and strafe values as the translation vector with max magnitude of 1
        Vector rawTrans = new Vector(Range.clip(Math.hypot(strafe, forward), 0, 1), Math.atan2(forward, strafe));

        boolean ignoreTrans = rawTrans.getMagnitude() < 0.05;
        boolean ignoreRotation = Math.abs(rotation) < 0.05;

        double rotationScalar = (ignoreRotation) ? 0 : rotation;

        Vector[] podVectors = new Vector[pods.length];

        for (int i = 0; i < pods.length; i++) {
            SwervePod pod = pods[i];

            Vector translationVector = ignoreTrans ? new Vector(0, 0) : rawTrans;

            //we want -rotationScalar because after the vector rotation, the vectors will be pointing counterclockwise,
            // which is what we want when rotation < 0 (joystick to left)
            Vector rotationVector = new Vector(-rotationScalar, Math.atan2(pod.getYOffset(), pod.getXOffset()));

            //this gets the perpendicular vector for the wheel
            rotationVector.rotateVector(Math.PI / 2);

            podVectors[i] = translationVector.plus(rotationVector);
        }

        //finding if any vector has magnitude > maxPowerScaling
        double maxMagnitude = maxPowerScaling;
        for (Vector podVector : podVectors) {
            //voltage compensation impl copied straight from mecanum basically
            if (voltageCompensation) {
                double voltageNormalized = getVoltageNormalized();
                podVector.times(voltageNormalized);
            }
            maxMagnitude = Math.max(maxMagnitude, podVector.getMagnitude());
        }

        for (int podNum = 0; podNum < pods.length; podNum++) {
            Vector finalVector = podVectors[podNum].times(maxPowerScaling / maxMagnitude); //Normalizing if necessary while preserving relative sizes

            //2*Pi-theta because servos have positive clockwise rotation, while our angles are counterclockwise
            pods[podNum].move(2 * Math.PI - finalVector.getTheta(), finalVector.getMagnitude(),
                    ignoreTrans && ignoreRotation, motorCachingThreshold, servoCachingThreshold);
        }
    }

    @Override
    public void updateConstants() {
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;
        this.maxPowerScaling = constants.maxPower; //inherited from Drivetrain, used by CustomDrivetrain
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.servoCachingThreshold = constants.servoCachingThreshold;
        this.voltageCompensation = constants.useVoltageCompensation; //inherited from Drivetrain
        this.nominalVoltage = constants.nominalVoltage; //inherited from Drivetrain
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;

        leftFrontPod = new SwervePod(hardwareMap, constants.leftFrontServoName, constants.leftFrontEncoderName,
                constants.leftFrontMotorName, constants.leftFrontTurnPID,
                constants.leftFrontMotorDirection, constants.leftFrontServoDirection,
                constants.leftFrontPodAngleOffsetDeg, constants.leftFrontPodXYOffsets);

        rightFrontPod = new SwervePod(hardwareMap, constants.rightFrontServoName, constants.rightFrontEncoderName,
                constants.rightFrontMotorName, constants.rightFrontTurnPID,
                constants.rightFrontMotorDirection, constants.rightFrontServoDirection,
                constants.rightFrontPodAngleOffsetDeg, constants.rightFrontPodXYOffsets);

        leftRearPod = new SwervePod(hardwareMap, constants.leftRearServoName, constants.leftRearEncoderName,
                constants.leftRearMotorName, constants.leftRearTurnPID,
                constants.leftRearMotorDirection, constants.leftRearServoDirection,
                constants.leftRearPodAngleOffsetDeg, constants.leftRearPodXYOffsets);

        rightRearPod = new SwervePod(hardwareMap, constants.rightRearServoName, constants.rightRearEncoderName,
                constants.rightRearMotorName, constants.rightRearTurnPID,
                constants.rightRearMotorDirection, constants.rightRearServoDirection,
                constants.rightRearPodAngleOffsetDeg, constants.rightRearPodXYOffsets);

        pods[0] = leftFrontPod;
        pods[1] = rightFrontPod;
        pods[2] = leftRearPod;
        pods[3] = rightRearPod;
    }

    @Override
    public void breakFollowing() {
        for (SwervePod pod : pods) {
            pod.setMotorPower(0);
            pod.setMotorToFloat();
            pod.disableServo();
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
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    @Override
    public String debugString() {
        return "Mecanum{" +
                " leftFront=" + leftFrontPod.debugString() +
                ", leftRear=" + leftRearPod.debugString() +
                ", rightFront=" + rightFrontPod.debugString() +
                ", rightRear=" + rightRearPod.debugString() +
                ", motorCachingThreshold=" + motorCachingThreshold +
                ", useBrakeModeInTeleOp=" + useBrakeModeInTeleOp +
                '}';
    }
}
