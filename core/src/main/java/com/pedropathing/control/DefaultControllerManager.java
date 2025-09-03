package com.pedropathing.control;

import com.pedropathing.follower.FollowerConstants;

/**
 * Controller manager for v2.0.0
 */
public class DefaultControllerManager implements ControllerManager {
    private DualController<PIDFController> translational;
    private DualController<FilteredPIDFController> drive;
    private DualController<PIDFController> heading;

    FollowerConstants constants;
    
    public DefaultControllerManager(FollowerConstants constants) {
        this.constants = constants;
    }
    
    @Override
    public Controller createTranslational() {
        return translational = new DualController<>(
            new PIDFController(constants.coefficientsTranslationalPIDF),
            new PIDFController(constants.coefficientsSecondaryTranslationalPIDF),
            constants.translationalPIDFSwitch,
            constants.useSecondaryTranslationalPIDF
        );
    }
    
    @Override
    public Controller createHeading() {
        return heading = new DualController<>(
            new PIDFController(constants.coefficientsHeadingPIDF),
            new PIDFController(constants.coefficientsSecondaryHeadingPIDF),
            constants.headingPIDFSwitch,
            constants.useSecondaryHeadingPIDF
        );
    }
    
    @Override
    public Controller createDrive() {
        return drive = new DualController<>(
            new FilteredPIDFController(constants.coefficientsDrivePIDF),
            new FilteredPIDFController(constants.coefficientsSecondaryDrivePIDF),
            constants.drivePIDFSwitch,
            constants.useSecondaryDrivePIDF
        );
    }
    
    @Override
    public void reset() {
        drive.reset();
        translational.reset();
        heading.reset();
    }

    @Override
    public void updateCoefficients() {
        translational.primaryController.setCoefficients(constants.coefficientsTranslationalPIDF);
        translational.secondaryController.setCoefficients(constants.coefficientsSecondaryTranslationalPIDF);
        drive.primaryController.setCoefficients(constants.coefficientsDrivePIDF);
        drive.secondaryController.setCoefficients(constants.coefficientsSecondaryDrivePIDF);
        heading.primaryController.setCoefficients(constants.coefficientsHeadingPIDF);
        heading.secondaryController.setCoefficients(constants.coefficientsSecondaryHeadingPIDF);
    }
}
