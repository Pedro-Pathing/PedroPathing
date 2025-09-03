package com.pedropathing.control;

import com.pedropathing.follower.FollowerConstants;

/**
 * @author Jacob Ophoven - 18535, Frozen Code
 */
public interface ControllerManager {
    Controller<?> createTranslational(FollowerConstants constants);
    Controller<?> createHeading(FollowerConstants constants);
    Controller<?> createDrive(FollowerConstants constants);
    
    void reset();
    void updateCoefficients(FollowerConstants constants);
    
    class DefaultControllerManager implements ControllerManager {
        private Controller<PIDFCoefficients> translational;
        private Controller<FilteredPIDFCoefficients> drive;
        private Controller<PIDFCoefficients> heading;
        
        // pass in constants in init instead of at funcs
        
        @Override
        public Controller<PIDFCoefficients> createTranslational(FollowerConstants constants) {
            PIDFController base = new PIDFController(constants.coefficientsTranslationalPIDF);
            
            if (constants.useSecondaryTranslationalPIDF) {
                translational = new DualController<>(
                    base,
                    constants.coefficientsTranslationalPIDF,
                    constants.coefficientsSecondaryTranslationalPIDF,
                    constants.translationalPIDFSwitch
                );
            } else {
                translational = base;
            }
            
            return translational;
        }
        
        @Override
        public Controller<PIDFCoefficients> createHeading(FollowerConstants constants) {
            PIDFController baseHeading = new PIDFController(constants.coefficientsHeadingPIDF);
            
            if (constants.useSecondaryHeadingPIDF) {
                heading = new DualController<>(
                    baseHeading,
                    constants.coefficientsHeadingPIDF,
                    constants.coefficientsSecondaryHeadingPIDF,
                    constants.headingPIDFSwitch
                );
            } else {
                heading = baseHeading;
            }
            
            return heading;
        }
        
        @Override
        public Controller<FilteredPIDFCoefficients> createDrive(FollowerConstants constants) {
            FilteredPIDFController baseDrive = new FilteredPIDFController(constants.coefficientsDrivePIDF);
            
            if (constants.useSecondaryDrivePIDF) {
                drive = new DualController<>(
                    baseDrive,
                    constants.coefficientsDrivePIDF,
                    constants.coefficientsSecondaryDrivePIDF,
                    constants.drivePIDFSwitch
                );
            } else {
                drive = baseDrive;
            }
            
            return drive;
        }
        
        @Override
        public void reset() {
            drive.reset();
            translational.reset();
            heading.reset();
        }

        @Override
        public void updateCoefficients(FollowerConstants constants) {
            translational.setCoefficients(constants.coefficientsTranslationalPIDF);
            drive.setCoefficients(constants.coefficientsDrivePIDF);
            heading.setCoefficients(constants.coefficientsHeadingPIDF);
        }
    }
    
    // Usage: new DualController<>(controller, primaryCoefficients, secondaryCoefficients, threshold);
    class DualController<C extends Coefficients<C, CT>, CT extends Controller<C>> implements Controller<C> {
        private final CT controller;
        private C primaryCoefficients;
        private C secondaryCoefficients;
        private final double threshold;
        
        public DualController(CT controller, C primaryCoefficients,
                              C secondaryCoefficients, double threshold) {
            this.controller = controller;
            this.primaryCoefficients = primaryCoefficients;
            this.secondaryCoefficients = secondaryCoefficients;
            this.threshold = threshold;
        }
        
        @Override
        public double run(double error) {
            if (Math.abs(error) < threshold) {
                controller.setCoefficients(primaryCoefficients);
            } else {
                controller.setCoefficients(secondaryCoefficients);
            }
            return controller.run(error);
        }
        
        @Override
        public void reset() {
            controller.reset();
        }
        
        @Override
        public void setCoefficients(C coefficients) {
            primaryCoefficients = coefficients;
        }
        
        public void setSecondaryCoefficients(C coefficients) {
            secondaryCoefficients = coefficients;
        }
    }
}
