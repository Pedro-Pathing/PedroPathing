package com.pedropathing.revhub.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CachedMotor {
    private final DcMotorEx motor;

    private double power = 0;
    private final double powerThreshold;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    private DcMotorSimple.Direction direction;

    public CachedMotor(DcMotorEx motor, double powerThreshold) {
        this.motor = motor;
        this.powerThreshold = powerThreshold;
    }

    public void setPower(double power) {
        if (Math.abs(this.power - power) >= powerThreshold) {
            this.power = power;
            motor.setPower(power);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (zeroPowerBehavior != behavior) {
            zeroPowerBehavior = behavior;
            motor.setZeroPowerBehavior(behavior);
        }
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        if (this.direction != direction) {
            this.direction = direction;
            motor.setDirection(direction);
        }
    }

    public DcMotorEx raw() {
        return motor;
    }
}