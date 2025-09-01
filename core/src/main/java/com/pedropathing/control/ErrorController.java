package com.pedropathing.control;

public interface ErrorController {
    double run();
    void reset();
    void setCoefficients(ControllerCoefficients coefficients);
}
