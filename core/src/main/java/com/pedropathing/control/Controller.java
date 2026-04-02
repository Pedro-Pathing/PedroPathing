package com.pedropathing.control;

public interface Controller<T> {
    T getCoefficients();
    void setCoefficients(T coefficients);
    double calculate(double error);
    void reset();
}