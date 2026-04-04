package com.pedropathing.control;

public class PIDFCoefficients {
    public double kP;
    public double kI;
    public double kD;
    public double kF;

    public PIDFCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
}