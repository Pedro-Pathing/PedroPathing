package com.pedropathing.control;

public class PIDFCoefficients extends PIDCoefficients {
    public double kF;

    public PIDFCoefficients(double kP, double kI, double kD, double kF) {
        super(kP, kI, kD);
        this.kF = kF;
    }
}