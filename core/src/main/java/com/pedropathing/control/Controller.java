package com.pedropathing.control;

public interface Controller<T> {
    void setCoefficients(T coefficients);
    double calculate(double error);
    void reset();
}