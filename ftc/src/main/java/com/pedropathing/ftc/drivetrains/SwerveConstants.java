package com.pedropathing.ftc.drivetrains;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Constants for swerve drive configuration
 *
 * @author Kabir Goyal - 365 MOE
 * @author Baron Henderson
 */
public class SwerveConstants {

    public double xVelocity = 80.0;
    public double yVelocity = 80.0;

    public boolean useBrakeModeInTeleOp = false;
    public double maxPower = 1.0;
    public boolean useVoltageCompensation = false;
    public double nominalVoltage = 12.0;
    public double staticFrictionCoefficient = 0.1;
    public double epsilon = 0.05;

    enum ZeroPowerBehavior {
        RESIST_MOVEMENT,
        IGNORE_ANGLE_CHANGES
    }

    public ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.IGNORE_ANGLE_CHANGES;

    public SwerveConstants() {
        defaults();
    }

    /**
     * @param velocity the max speed in ANY direction because swerve can do that :)
     * @return
     */
    public SwerveConstants velocity(double velocity) {
        this.xVelocity = velocity;
        this.yVelocity = velocity;
        return this;
    }

    public SwerveConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public SwerveConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public SwerveConstants useBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
        return this;
    }

    public SwerveConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public SwerveConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public SwerveConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public SwerveConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
    }

    public SwerveConstants epsilon(double epsilon) {
        this.epsilon = epsilon;
        return this;
    }

    public SwerveConstants zeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
        return this;
    }

    public double getVelocity() {
        return Math.max(getXVelocity(), getYVelocity());
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public boolean getUseBrakeModeInTeleOp() {
        return useBrakeModeInTeleOp;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public boolean getUseVoltageCompensation() {
        return useVoltageCompensation;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    public double getEpsilon() {
        return epsilon;
    }

    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    public void setVelocity(double velocity) {
        this.xVelocity = velocity;
        this.yVelocity = velocity;
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public void setUseBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void setUseVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    public void setStaticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
    }

    public void setEpsilon(double epsilon) {
        this.epsilon = epsilon;
    }

    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    public void defaults() {
        xVelocity = 80.0;
        yVelocity = 80.0;
        useBrakeModeInTeleOp = false;
        maxPower = 1.0;
        useVoltageCompensation = false;
        nominalVoltage = 12.0;
        staticFrictionCoefficient = 0.1;
        epsilon = 0.05;
        zeroPowerBehavior = ZeroPowerBehavior.IGNORE_ANGLE_CHANGES;
    }
}