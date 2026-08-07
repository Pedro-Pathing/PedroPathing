package com.pedropathing.revhub.drivetrains;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Pair;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
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
public class Swerve implements Drivetrain {

    protected double lastHeading = 0;

    private boolean manualBrakeMode, voltageCompensation;
    private double staticFrictionCoefficient;
    private double nominalVoltage;
    private SwerveConfig.ZeroPowerBehavior zeroPowerBehavior;
    private double epsilon;

    private List<SwervePod> pods;

    private double lastForward = 0;
    private double lastStrafe = 0;
    private double lastRotation = 0;
    private double lastAvgScaling = 0;

    private final VoltageSensor voltageSensor;

    /**
     * @param pods SwervePods, coaxial or differential
     */
    public Swerve(HardwareMap hardwareMap, SwerveConfig config, SwervePod... pods) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        manualBrakeMode = config.manualBrakeMode.get();
        voltageCompensation = config.voltageCompensation.get();
        zeroPowerBehavior = config.zeroPowerBehavior.get();
        epsilon = config.epsilon.get();
        staticFrictionCoefficient = config.staticFrictionCoefficient.get();
        nominalVoltage = config.nominalVoltage.get();
        this.pods = Arrays.asList(pods);
    }

    /**
     * Stops following and holds pod angles while floating drive motors.
     */
    @Override
    public void stop() {
        for (SwervePod pod : pods) {
            pod.move(pod.getAngle(), 0, true);
            pod.setToFloat();
        }
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        if (manual && manualBrakeMode)
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        else
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        applyDrive(powers);
    }

    public void applyDrive(DrivePowers powers) {
        double forward = powers.forward();
        double strafe = -powers.strafe();
        double rotation = -powers.turn();

        lastForward = forward;
        lastStrafe = strafe;
        lastRotation = rotation;

        boolean zeroTrans = Math.hypot(strafe, forward) < epsilon;
        boolean zeroRotation = Math.abs(rotation) < epsilon;

        Vector2D[] podVectors = computePodPowers(powers);

        // finding if any vector has magnitude > maxPowerScaling
        double maxMagnitude = 1;
        for (Vector2D podVector : podVectors) {
            if (voltageCompensation) {
                double voltageNormalized = getVoltageNormalized();
                podVector.times(voltageNormalized);
            }
            maxMagnitude = Math.max(maxMagnitude, podVector.magnitude());
        }

        // Find the avg scaling constant (avg of cos(angle error))
        double avgScaling = 0;

        for (int i = 0; i < pods.size(); i++) {
            double currentRad = pods.get(i).getAngle();

            // ask the pod to translate the wheel-space theta into the encoder frame
            double targetRad = pods.get(i).adjustThetaForEncoder(podVectors[i].theta());

            // compute shortest signed error in radians using MathFunctions
            double mag = Angle.smallestDifference(currentRad, targetRad);
            double dir = Angle.turnDirection(currentRad, targetRad);
            double errorRad = (mag == Math.PI) ? -Math.PI : mag * dir;

            avgScaling += Math.abs(Math.cos(errorRad));
        }

        avgScaling /= pods.size();
        lastAvgScaling = avgScaling;

        for (int podNum = 0; podNum < pods.size(); podNum++) {
            // Normalizing if necessary while preserving relative sizes
            Vector2D finalVector = podVectors[podNum].times(1 / maxMagnitude);

            pods.get(podNum).move(finalVector.theta(), finalVector.magnitude() * avgScaling,
                    zeroTrans && zeroRotation && zeroPowerBehavior == SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
        }
    }

    public Vector2D[] computePodPowers(DrivePowers powers) {
        double forward = powers.forward();
        double strafe = -powers.strafe();
        double rotation = -powers.turn();

        Vector2D[] podVectors = new Vector2D[pods.size()];
        Vector2D rawTrans = Vector2D.polar(Range.clip(Math.hypot(strafe, forward), 0, 1), Math.atan2(forward, strafe));

        boolean zeroTrans = rawTrans.magnitude() < epsilon;
        boolean zeroRotation = Math.abs(rotation) < epsilon;

        double rotationScalar = (zeroRotation) ? 0 : rotation;

        for (int i = 0; i < pods.size(); i++) {
            SwervePod pod = pods.get(i);

            Vector2D translationVector = zeroTrans ? Vector2D.zero() : rawTrans;

            Vector2D rotationVector = Vector2D.polar(rotationScalar, Math.atan2(-pod.getOffset().y(), pod.getOffset().x()))
                    .rotate(Math.PI / 2);

            podVectors[i] = translationVector.plus(rotationVector);
            if (zeroPowerBehavior == SwerveConfig.ZeroPowerBehavior.X_LOCK
                    && zeroTrans && zeroRotation) {
                rotationVector.rotate(-Math.PI / 2);
                podVectors[i] = rotationVector;
            }
        }

        return podVectors;
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        double lambda = 1.0;

        Vector2D[] currentPowers = computePodPowers(current);
        Vector2D[] deltaPowers = computePodPowers(delta);

        for (int i = 0; i < currentPowers.length; i++) {
            Vector2D a = currentPowers[i];
            Vector2D b = deltaPowers[i];

            double quadraticTerm = b.magnitudeSquared();
            double linearTerm = 2 * a.dot(b);
            double constantTerm = a.magnitudeSquared() - 1;

            Pair<Double, Double> wheelSolution = Utils.solveQuadratic(quadraticTerm, linearTerm, constantTerm);

            double t1 =  wheelSolution.first();
            double t2 = wheelSolution.second();

            if (t1 >= 0.0 && t1 < lambda) lambda = t1;
            if (t2 >= 0.0 && t2 < lambda) lambda = t2;
        }

        return lambda;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (SwervePod pod : pods) {
            if (behavior == DcMotor.ZeroPowerBehavior.BRAKE)
                pod.setToBreak();
            else
                pod.setToFloat();
        }
    }

    /**
     * @return static friction coefficient used for voltage compensation
     */
    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    /**
     * @return normalized voltage for voltage compensation
     */
    private double getVoltageNormalized() {
        double voltage = voltageSensor.getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage
                - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }
}

