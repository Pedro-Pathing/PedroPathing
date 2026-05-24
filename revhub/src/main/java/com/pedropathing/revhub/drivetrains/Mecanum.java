package com.pedropathing.revhub.drivetrains;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mecanum implements Drivetrain {
    public final double strafingEffortMultiplier;
    private final boolean manualBrakeMode;

    private final CachedMotor[] motors;
    private final double[] wheelPowers = new double[4];

    private static final int FL = 0;
    private static final int BL = 1;
    private static final int FR = 2;
    private static final int BR = 3;
    private boolean manual;

    public Mecanum(HardwareMap map, MecanumConfig config) {
        double powerDeadband = config.powerDeadband.get();
        motors = new CachedMotor[]{
                new CachedMotor(map.get(DcMotorEx.class, config.leftFrontName.get()), powerDeadband),
                new CachedMotor(map.get(DcMotorEx.class, config.leftRearName.get()), powerDeadband),
                new CachedMotor(map.get(DcMotorEx.class, config.rightFrontName.get()), powerDeadband),
                new CachedMotor(map.get(DcMotorEx.class, config.rightRearName.get()), powerDeadband)
        };

        motors[FL].setDirection(config.leftFrontDirection.get());
        motors[BL].setDirection(config.leftRearDirection.get());
        motors[FR].setDirection(config.rightFrontDirection.get());
        motors[BR].setDirection(config.rightRearDirection.get());

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        manualBrakeMode = config.manualBrakeMode.get();

        strafingEffortMultiplier = config.maxForwardVelocity.get() / config.maxStrafeVelocity.get();
    }

    public void applyDrive(DrivePowers powers) {
        double upRight = -powers.strafe() * strafingEffortMultiplier + powers.forward();
        double downLeft = -powers.strafe() * strafingEffortMultiplier - powers.forward();

        wheelPowers[FL] = upRight - powers.turn();
        wheelPowers[BL] = downLeft + powers.turn();
        wheelPowers[FR] = downLeft - powers.turn();
        wheelPowers[BR] = upRight + powers.turn();

        Utils.Control.desaturate(wheelPowers);

        for (int i = 0; i < wheelPowers.length; i++) {
            motors[i].setPower(wheelPowers[i]);
        }
    }


    @Override
    public void drive(DrivePowers powers) {
        if (manual) {
            if (manualBrakeMode)
                setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            manual = false;
        }
        applyDrive(powers);
    }

    @Override
    public void manual(DrivePowers powers) {
        if (!manual) {
            if (manualBrakeMode)
                setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            manual = true;
        }
        applyDrive(powers);
    }

    @Override
    public void stop() {
        for (CachedMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (CachedMotor motor : motors) {
            motor.setZeroPowerBehavior(behavior);
        }
    }
}