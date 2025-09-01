package com.pedropathing.control;

public interface Controller {
    double run();
    void reset();
    void updateError(double error);
    void updatePosition(double position);
}
