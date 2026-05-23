package com.pedropathing.revhub.drivetrains;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mecanum implements Drivetrain {
    // TODO: look into making a cached motor class
    public final double strafingEffortMultiplier;
    private final boolean manualBrakeMode;

    private final DcMotorEx[] motors;
    private final double[] wheelPowers = new double[4];

    private static final int FL = 0;
    private static final int BL = 1;
    private static final int FR = 2;
    private static final int BR = 3;
    private boolean manual;

    public Mecanum(HardwareMap map, MecanumConfig config) {
        motors = new DcMotorEx[]{
                map.get(DcMotorEx.class, config.leftFrontName.get()),
                map.get(DcMotorEx.class, config.leftRearName.get()),
                map.get(DcMotorEx.class, config.rightFrontName.get()),
                map.get(DcMotorEx.class, config.rightRearName.get())
        };

        motors[FL].setDirection(config.leftFrontDirection.get());
        motors[BL].setDirection(config.leftRearDirection.get());
        motors[FR].setDirection(config.rightFrontDirection.get());
        motors[BR].setDirection(config.rightRearDirection.get());

        setMotorsFloat();

        manualBrakeMode = config.manualBrakeMode.get();

        strafingEffortMultiplier = config.maxForwardVelocity.get() / config.maxStrafeVelocity.get();
    }

    @Override
    public void drive(DrivePowers powers) {
        if (manual) {
            if (manualBrakeMode)
                setMotorsFloat();
            manual = false;
        }

        double upRight = -powers.strafe() * strafingEffortMultiplier + powers.forward();
        double downLeft = -powers.strafe() * strafingEffortMultiplier - powers.forward();

        wheelPowers[FL] = upRight - powers.turn();
        wheelPowers[BL] = downLeft + powers.turn();
        wheelPowers[FR] = downLeft - powers.turn();
        wheelPowers[BR] = upRight + powers.turn();

        double max = 0;
        for (double power : wheelPowers) {
            max = Math.max(max, Math.abs(power));
        }

        double scale = max > 1.0 ? 1 / max : 1;
        for (int i = 0; i < wheelPowers.length; i++) {
            double power = wheelPowers[i] * scale;
            if (power != motors[i].getPower()) {
                motors[i].setPower(power * scale);
            }
        }
    }

    @Override
    public void manual(DrivePowers powers) {
        if (!manual) {
            if (manualBrakeMode)
                setMotorsBrake();
            manual = true;
        }
        drive(powers);
    }

    @Override
    public void stop() {
        for (int i = 0; i < wheelPowers.length; i++) {
            motors[i].setPower(0);
        }
    }

    public void setMotorsFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setMotorsBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}