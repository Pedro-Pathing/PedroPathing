package com.pedropathing.control;

import com.pedropathing.follower.FollowerConstants;

/**
 * @author Jacob Ophoven - 18535, Frozen Code
 */
public interface ControllerManager {
    Controller createTranslational();
    Controller createHeading();
    Controller createDrive();
    
    void reset();
    void updateCoefficients();
}
