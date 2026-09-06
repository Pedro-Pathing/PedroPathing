package com.pedropathing.revhub.drivetrains;

import android.annotation.SuppressLint;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;

public class Mecanum implements Drivetrain {
    private final boolean manualBrakeMode;

    private final CachedMotor[] motors;
    public final double[] wheelPowers = new double[4];

    private static final int FL = 0;
    private static final int FR = 1;
    private static final int BL = 2;
    private static final int BR = 3;

    private double powerScale = 1.0;
    private DrivePowers drivePowers = DrivePowers.zero();

    public Mecanum(HardwareMap map, MecanumConfig config) {
        double powerDeadband = config.powerThreshold.get();
        
        motors = new CachedMotor[]{
                new CachedMotor(map.get(DcMotorEx.class, config.frontLeftName.get()), powerDeadband),
                new CachedMotor(map.get(DcMotorEx.class, config.frontRightName.get()), powerDeadband),
                new CachedMotor(map.get(DcMotorEx.class, config.backLeftName.get()), powerDeadband),
                new CachedMotor(map.get(DcMotorEx.class, config.backRightName.get()), powerDeadband)
        };

        motors[FL].setDirection(config.frontLeftDirection.get());
        motors[FR].setDirection(config.frontRightDirection.get());
        motors[BL].setDirection(config.backLeftDirection.get());
        motors[BR].setDirection(config.backRightDirection.get());

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        manualBrakeMode = config.manualBrakeMode.get();
    }

    @SuppressLint("DefaultLocale")
    public void applyDrive(DrivePowers powers) {
        drivePowers = powers;

        double[] wheelPowers = computeWheelPowersUnnormalized(powers);

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

    public double[] computeWheelPowersUnnormalized(DrivePowers powers) {
        double[] wheelPowers = new double[4];
        double forward = powers.forward();
        double strafe = powers.strafe();
        double turn = powers.turn();

        double fl = forward - strafe - turn;
        double fr = forward + strafe + turn;
        double bl = forward + strafe - turn;
        double br = forward - strafe + turn;

        wheelPowers[FL] = fl;
        wheelPowers[FR] = fr;
        wheelPowers[BL] = bl;
        wheelPowers[BR] = br;

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
        if (manual && manualBrakeMode)
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
    public String debugString() {
        return "Forward: " + drivePowers.forward() + "\n" +
                "Strafe: " + drivePowers.strafe() + "\n" +
                "Turn: " + drivePowers.turn() + "\n" +
                "Power Scale: " + powerScale + "\n" +
                "Wheel Powers: [" +
                wheelPowers[FL] + ", " +
                wheelPowers[FR] + ", " +
                wheelPowers[BL] + ", " +
                wheelPowers[BR] + "]";
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (CachedMotor motor : motors) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    /** Returns the sum of the four motors current in Amps
     * This is not bulk cached by the motors so each motor request is a hardware read
     */
    public double currentAmps() {
        double total = 0;
        for (CachedMotor motor : motors) {
            total += motor.raw().getCurrent(CurrentUnit.AMPS);
        }
        return total;
    }

    @Override
    public double interpolateVelocity(double xRadius, double yRadius, double theta) {
        return 1.0 / (Math.abs(Math.cos(theta)) / xRadius + Math.abs(Math.sin(theta)) / yRadius);
    }
}