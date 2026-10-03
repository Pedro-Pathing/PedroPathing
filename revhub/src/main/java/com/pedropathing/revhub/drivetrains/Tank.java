package com.pedropathing.revhub.drivetrains;

import android.annotation.SuppressLint;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.HashMap;
import java.util.Map;

public class Tank implements Drivetrain {
    private final TankConfig config;
    private final VoltageSensor voltageSensor;

    private final CachedMotor[] motors;
    public final double[] wheelPowers = new double[4];

    private static final int FL = 0;
    private static final int FR = 1;
    private static final int BL = 2;
    private static final int BR = 3;

    private double powerScale = 1.0;
    private DrivePowers drivePowers = DrivePowers.zero();

    public Tank(HardwareMap hardwareMap, TankConfig config) {
        this.config = config;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        double powerDeadband = config.powerThreshold.get();

        motors = new CachedMotor[]{
                new CachedMotor(hardwareMap.get(DcMotorEx.class, config.leftFrontName.get()), powerDeadband),
                new CachedMotor(hardwareMap.get(DcMotorEx.class, config.leftRearName.get()), powerDeadband),
                new CachedMotor(hardwareMap.get(DcMotorEx.class, config.rightFrontName.get()), powerDeadband),
                new CachedMotor(hardwareMap.get(DcMotorEx.class, config.rightRearName.get()), powerDeadband)
        };

        motors[FL].setDirection(config.leftFrontDirection.get());
        motors[BL].setDirection(config.leftRearDirection.get());
        motors[FR].setDirection(config.rightFrontDirection.get());
        motors[BR].setDirection(config.rightRearDirection.get());

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @SuppressLint("DefaultLocale")
    public void applyDrive(DrivePowers powers) {
        drivePowers = powers;

        double[] wheelPowers = computeWheelPowersUnnormalized(powers);

        if (config.voltageCompensation.get()) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] *= voltageNormalized;
            }
        }

        double maxPower = 1.0;
        for (double power : wheelPowers) {
            maxPower = Math.max(maxPower, Math.abs(power));
        }

        powerScale = 1.0 / maxPower;

        for (int i = 0; i < wheelPowers.length; i++) {
            this.wheelPowers[i] = wheelPowers[i] / maxPower;
            motors[i].setPower(this.wheelPowers[i]);
        }
    }

    /** Tank drive has no strafe axis; powers.strafe() is ignored. */
    public double[] computeWheelPowersUnnormalized(DrivePowers powers) {
        double[] wheelPowers = new double[4];
        double forward = powers.forward();
        double turn = powers.turn();

        double left = forward + turn;
        double right = forward - turn;

        wheelPowers[FL] = left;
        wheelPowers[BL] = left;
        wheelPowers[FR] = right;
        wheelPowers[BR] = right;

        return wheelPowers;
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        double lambda = 1.0;

        double[] currentPowers = computeWheelPowersUnnormalized(current);
        double[] deltaPowers = computeWheelPowersUnnormalized(delta);

        for (int i = 0; i < 4; i++) {
            double a = currentPowers[i];
            double b = deltaPowers[i];

            if (Math.abs(b) < 1e-9) continue;

            double t1 = (1.0 - a) / b;
            double t2 = (-1.0 - a) / b;

            if (t1 >= 0.0 && t1 < lambda) lambda = t1;
            if (t2 >= 0.0 && t2 < lambda) lambda = t2;
        }

        return Utils.clamp(lambda, 0.0, 1.0);
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        if (manual && config.manualBrakeMode.get())
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        else
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        applyDrive(powers);
    }

    @Override
    public void stop() {
        for (CachedMotor motor : motors) {
            motor.setPower(0);
        }
    }

    @Override
    public void stop(boolean brake) {
        if (brake) {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        stop();
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = new HashMap<>();

        map.put("forward", drivePowers.forward());
        map.put("strafe", drivePowers.strafe());
        map.put("turn", drivePowers.turn());
        map.put("powerScale", powerScale);
        map.put("leftFrontWheelPower", wheelPowers[FL]);
        map.put("rightFrontWheelPower", wheelPowers[FR]);
        map.put("leftBackWheelPower", wheelPowers[BL]);
        map.put("rightBackWheelPower", wheelPowers[BR]);
        return map;
    }

    @Override
    public double interpolateVelocity(double xRadius, double yRadius, double theta) {
        return Math.max(0, Math.cos(theta)) * xRadius;
    }

    @Override
    public double interpolateAcceleration(double xRadius, double yRadius, double theta) {
        return Math.max(0, Math.cos(theta)) * xRadius;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (CachedMotor motor : motors) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        double nominalVoltage = config.nominalVoltage.get();
        double staticFrictionCoefficient = config.staticFrictionCoefficient.get();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage
                - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    /** Returns the sum of the four motors' current in Amps.
     * This is not bulk cached by the motors so each motor request is a hardware read.
     */
    public double currentAmps() {
        double total = 0;
        for (CachedMotor motor : motors) {
            total += motor.raw().getCurrent(CurrentUnit.AMPS);
        }
        return total;
    }
}