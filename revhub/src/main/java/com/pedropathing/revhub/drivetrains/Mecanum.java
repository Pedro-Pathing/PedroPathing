package com.pedropathing.revhub.drivetrains;

import android.annotation.SuppressLint;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.util.RobotLog;

public class Mecanum implements Drivetrain {
    private final boolean manualBrakeMode;

    private final CachedMotor[] motors;
    private final double[] wheelPowers = new double[4];

    private static final int FL = 0;
    private static final int BL = 1;
    private static final int FR = 2;
    private static final int BR = 3;

    public Mecanum(HardwareMap map, MecanumConfig config) {
        double powerDeadband = config.powerThreshold.get();
        
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
    }

    @SuppressLint("DefaultLocale")
    public void applyDrive(DrivePowers powers) {
        double forward = powers.forward();
        double strafe = powers.strafe();
        double turn = powers.turn();

        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;
        
        // Normalize by the largest absolute value (or 1) so we preserve ratios but
        // guarantee outputs stay in [-1, 1]. This is preferable to summing abs inputs.
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));

        wheelPowers[FL] = fl / max;
        wheelPowers[BL] = bl / max;
        wheelPowers[FR] = fr / max;
        wheelPowers[BR] = br / max;

        for (int i = 0; i < wheelPowers.length; i++) {
            motors[i].setPower(wheelPowers[i]);
        }

        boolean normalized = max > 1.0;
        RobotLog.i("Mecanum", String.format(
                "Mecanum drive raw: fl=%.3f bl=%.3f fr=%.3f br=%.3f | normMax=%.3f | normalized=%b | out: fl=%.3f bl=%.3f fr=%.3f br=%.3f",
                fl, bl, fr, br, max, normalized,
                wheelPowers[FL], wheelPowers[BL], wheelPowers[FR], wheelPowers[BR]
        ));
    }


    @Override
    public void drive(DrivePowers powers, boolean manual) {
        if (manual) {
            if (manualBrakeMode)
                setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
}