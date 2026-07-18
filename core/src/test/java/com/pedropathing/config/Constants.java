package com.pedropathing.config;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;

public class Constants {
    public static double kP = 0.03;
    public static double headingFF = 0.065;
    public static double velocityFF = 0.009;
    public static double accelFF = 0.001;

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller largeTranslationalForward = Controller.pid(0.07,0,0).plus(Controller.staticFeedforward(0.01));
                Controller smallTranslationalForward = Controller.pid(0.07,0,0);
                Controller smallTranslationalLateral = Controller.pid(.12,0,0).plus(Controller.staticFeedforward(0.0005));
                Controller largeTranslationalLateral = Controller.pid(.12,0,0).plus(Controller.staticFeedforward(0.01));
                c.forwardTranslationalController.set(Controller.piecewise(Controller.staticFeedforward(0)).add(0.5, smallTranslationalForward).add(2.5, largeTranslationalForward));
                c.lateralTranslationalController.set(Controller.piecewise(Controller.staticFeedforward(0)).add(0.5, smallTranslationalLateral).add(2.5, largeTranslationalLateral));
                c.brakeController.set(Controller.pid(kP, 0, 0).plus(Controller.dynamicFeedforward(velocityFF)));
                c.brakeAccelFeedforward.set(Controller.dynamicFeedforward(accelFF));
                c.maxBrakingPower.set(0.3);
                Controller largeHeading = Controller.pid(1.2889, 0, 0.1899).plus(Controller.staticFeedforward(0.01));
                c.headingController.set(largeHeading);
                c.linearBrakeCoefficients.set(Matrix.diag(0.0633, 0.0633));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.00146, 0.00146));
                c.headingFeedforward.set(Controller.dynamicFeedforward(headingFF));
                c.centripetalScaling.set(0.0005);
                c.fullPowerCoast.set(true);
                c.cosineScale.set(false);
            }
    );
}
