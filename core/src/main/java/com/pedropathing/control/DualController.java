package com.pedropathing.control;

class DualController<C extends Controller> implements Controller {
    final C primaryController;
    final C secondaryController;
    double threshold;
    boolean secondaryEnabled;
    
    public DualController(C primaryController, C secondaryController,
                          double threshold, boolean secondaryEnabled) {
        this.primaryController = primaryController;
        this.secondaryController = secondaryController;
        this.threshold = threshold;
        this.secondaryEnabled = secondaryEnabled;
    }
    
    @Override
    public double run(double error) {
        if (secondaryEnabled && Math.abs(error) > threshold) {
            return secondaryController.run(error);
        }
        return primaryController.run(error);
    }
    
    @Override
    public void reset() {
        primaryController.reset();
        secondaryController.reset();
    }
}
