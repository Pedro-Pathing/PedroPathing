package com.pedropathing.ftc.drivetrains;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.drivetrain.CustomDrivetrain;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;


/**
 * Swerve drivetrain implementation
 * @author Kabir Goyal - 365 MOE
 */
public class Swerve extends Drivetrain {
    public SwerveConstants constants;

    protected Vector lastTranslationalVector = new Vector();
    protected Vector lastHeadingPower = new Vector();
    protected Vector lastCorrectivePower = new Vector();
    protected Vector lastPathingPower = new Vector();
    protected double lastHeading = 0;

    private boolean useBrakeModeInTeleOp; //implemented
    private double motorCachingThreshold;
    private double servoCachingThreshold;
    private double staticFrictionCoefficient;

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

    public Swerve(HardwareMap hardwareMap, SwerveConstants constants)  {
        this.hardwareMap = hardwareMap;
        this.constants = constants;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        pods = new SwervePod[4];
        updateConstants();

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

    /**
     * This method takes in forward, strafe, and rotation values and applies them to the drivetrain.
     * Intended to work exactly like an arcade drive would in a typical TeleOp, this method can be a copy pasted from
     * a robot-centric arcade drive implementation.
     *
     * @param forward the forward power value, which would typically be -gamepad1.left_stick_y in a normal arcade drive setup.
     * @param strafe the strafe power value, which would typically be gamepad1.left_stick_x in a normal arcade drive setup.
     * @param rotation the rotation power value, which would typically be gamepad1.right_stick_x in a normal arcade drive setup.
     */
    public void arcadeDrive(double forward, double strafe, double rotation) {
        strafe*=-1;

        lastForward = forward;
        lastStrafe = strafe;
        lastRotation = rotation;

        //stores forward and strafe values as the translation vector with max magnitude of 1
        Vector rawTrans = new Vector(Range.clip(Math.hypot(strafe, forward), 0, 1), Math.atan2(forward, strafe));

        boolean ignoreTrans = rawTrans.getMagnitude() < 0.05000000000000001;
        boolean ignoreRotation = Math.abs(rotation) < 0.05000000000000001;

        double rotationScalar = (ignoreRotation) ? 0 : rotation;

        Vector[] podVectors = new Vector[pods.length];

        for (int i = 0; i < pods.length; i++) {
            SwervePod pod = pods[i];

            Vector translationVector = ignoreTrans ? new Vector(0, 0) : rawTrans;

            //actually positive rotation scalar because positive turning is to the left
            Vector rotationVector = new Vector(rotationScalar, Math.atan2(pod.getYOffset(), pod.getXOffset()));

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

        //Find the avg scaling constant (avg of cos(angle error))
        double avgScaling = 0;

        for (int i = 0; i < pods.length; i++) {
            double currentAngle = pods[i].getAngleAfterOffsetDeg();
            //might need 2pi-theta here
            double target = SwervePod.normalizeNeg180To180(Math.toDegrees(2*Math.PI-podVectors[i].getTheta()) + 90);

            double error = SwervePod.getError(currentAngle, target);

            avgScaling += Math.abs(Math.cos(Math.toRadians(error)));
        }

        avgScaling /= 4;
        lastAvgScaling = avgScaling;



        for (int podNum = 0; podNum < pods.length; podNum++) {
            Vector finalVector = podVectors[podNum].times(maxPowerScaling / maxMagnitude); //Normalizing if necessary while preserving relative sizes

            //2*Pi-theta because servos have positive clockwise rotation, while our angles are counterclockwise
            pods[podNum].move(2 * Math.PI - finalVector.getTheta(), finalVector.getMagnitude()  * avgScaling,
                    ignoreTrans && ignoreRotation, motorCachingThreshold, servoCachingThreshold); //seing if motor / servo caching is an issue
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
    }

    @Override
    public void breakFollowing() {
        for (SwervePod pod : pods) {
            pod.setMotorPower(0);
            pod.setMotorToFloat();
//            pod.disableServo(); yea we don't want to do this lmao
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
        return "Swerve{" +
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
//                ", leftFront=" + leftFrontPod.debugString() +
//                ", rightFront=" + rightFrontPod.debugString() +
//                ", rightRear=" + rightRearPod.debugString() +
                "\n}";
    }

//    @Override
//    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
//        double correctivePowerCap = 0.4;
//
//        // clamps down the magnitudes of the input vectors, now including correctivePowerCap
//        if (correctivePower.getMagnitude() >= correctivePowerCap * maxPowerScaling) {
//            correctivePower.setMagnitude(correctivePowerCap * maxPowerScaling);
//        }
//
//        if (headingPower.getMagnitude() > maxPowerScaling)
//            headingPower.setMagnitude(maxPowerScaling);
//        if (pathingPower.getMagnitude() > maxPowerScaling)
//            pathingPower.setMagnitude(maxPowerScaling);
//
//
//        double originalMagnitude = headingPower.plus(pathingPower).getMagnitude();
//
//        //intended to be the additional weighting of heading over pathing, 1.0 just means adding them
//        double headingPowerWeighting = 1.0;
//        headingPower = headingPower.times(headingPowerWeighting);
//
//        double pathingScalingFactor = Math.min(
//                findNormalizingScaling(headingPower, pathingPower, originalMagnitude),
//                findNormalizingScaling(headingPower, pathingPower.times(-1), originalMagnitude)
//        );
//
//        pathingPower = pathingPower.times(pathingScalingFactor);
//
//        Vector combinedVector = headingPower.plus(pathingPower);
//
//        if (correctivePower.plus(combinedVector).getMagnitude() >= maxPowerScaling) {
//            // Scale down the combined vector
//            double scalingFactor = findNormalizingScaling(correctivePower, combinedVector, maxPowerScaling);
//
//            // Scale the individual components
//            headingPower = headingPower.times(scalingFactor);
//            pathingPower = pathingPower.times(scalingFactor);
//        }
//
//
//
//        Vector finalTranslation = correctivePower.plus(pathingPower);
//
//        return new double[] {
//                finalTranslation.getXComponent(),
//                finalTranslation.getYComponent(),
//                headingPower.dot(new Vector(1, robotHeading))
//        };
//    }
    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() >= maxPowerScaling) {
            correctivePower.setMagnitude(maxPowerScaling);
            return new double[] {
                    correctivePower.getXComponent(),
                    correctivePower.getYComponent(),
                    0
            };
        }

        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        if (scaleDown(correctivePower, headingPower, true)) {
            headingPower = scaledVector(correctivePower, headingPower, true);
            return new double[] {
                    correctivePower.getXComponent(),
                    correctivePower.getYComponent(),
                    headingPower.dot(new Vector(1, robotHeading))
            };
        } else {
            Vector combinedStatic = correctivePower.plus(headingPower);
            if (scaleDown(combinedStatic, pathingPower, false)) {
                pathingPower = scaledVector(combinedStatic, pathingPower, false);
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        combinedMovement.getYComponent(),
                        headingPower.dot(new Vector(1, robotHeading))
                };
            } else {
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        combinedMovement.getYComponent(),
                        headingPower.dot(new Vector(1, robotHeading))
                };
            }
        }
    }

    private boolean scaleDown(Vector staticVector, Vector variableVector, boolean useMinus) {
        return (staticVector.plus(variableVector).getMagnitude() >= maxPowerScaling) ||
                (useMinus && staticVector.minus(variableVector).getMagnitude() >= maxPowerScaling);
    }

    private Vector scaledVector(Vector staticVector, Vector variableVector, boolean useMinus) {
        double scalingFactor = useMinus? Math.min(findNormalizingScaling(staticVector, variableVector, maxPowerScaling),
                findNormalizingScaling(staticVector, variableVector.times(-1), maxPowerScaling)) :
                findNormalizingScaling(staticVector, variableVector, maxPowerScaling);
        return variableVector.times(scalingFactor);
    }

    @Override
    public void runDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        double[] calculatedDrive = calculateDrive(correctivePower, headingPower, pathingPower, robotHeading);
        Vector translationalVector = new Vector();
        translationalVector.setOrthogonalComponents(calculatedDrive[0], calculatedDrive[1]);
        lastPathingPower = pathingPower;
        lastCorrectivePower = correctivePower;

        lastTranslationalVector = translationalVector; //before rotation
        lastHeadingPower = headingPower;
        lastHeading = robotHeading;

        translationalVector.rotateVector(-robotHeading); // this should make it field centric when field centric is desired and robot centric otherwise
        arcadeDrive(translationalVector.getXComponent(), translationalVector.getYComponent(), calculatedDrive[2]);
    }


    @Deprecated
    @Override
    public void runDrive(double[] drivePowers) {}
}