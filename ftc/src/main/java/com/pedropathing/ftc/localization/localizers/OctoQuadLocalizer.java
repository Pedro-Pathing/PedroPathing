/*
 * Copyright (c) 2025 DigitalChickenLabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OctoQuadLocalizer implements Localizer
{
    protected final OctoQuad octoQuad;
    protected final OctoQuad.LocalizerDataBlock localizerData = new OctoQuad.LocalizerDataBlock();

    protected int headingWraps = 0;
    protected double integratedHeading;
    protected float lastNormalizedHeading = 0;

    protected Pose currentVelocity = new Pose();
    protected Pose currentPose = new Pose();

    protected DataSupplier externalDataSupplier = null;

    /**
     * Controls the hardware initialization behavior in the constructor
     */
    public enum InitMode
    {
        /**
         * Initialize the hardware, e.g. send all localizer constants & reset IMU
         */
        INITIALIZE_OCTOQUAD,

        /**
         * Do NOT perform any initialization on the hardware: any values supplied
         * to the constructor in the {@link OctoQuadConstants} object will be ignored!!
         */
        ASSUME_EXTERNAL_INITIALIZATION
    }

    /**
     * Allows decoupling OctoQuad hardware read cycle from PedroPathing.
     * This may be useful if you, for example, want to use a single bulk
     * read from the OQ to grab both localizer and encoder data, and then
     * send the localizer data to Pedro while using the encoder data elsewhere.
     */
    public interface DataSupplier
    {
        /**
         * This will be called when Pedro wants new localization data.
         * @return most recent localization data that you have read from
         *         the OctoQuad externally in your OpMode.
         */
        OctoQuad.LocalizerDataBlock onDataRequest();
    }

    /**
     * Set a callback to provide new localization data externally,
     * rather than having Pedro poll the hardware itself.
     * See corresponding comments on {@link DataSupplier}
     * @param supplier callback to provide new localization data externally
     */
    public void setExternalDataSupplier(DataSupplier supplier)
    {
        this.externalDataSupplier = supplier;
    }

    /**
     * Construct a new Pedro Localizer instance for use with an OctoQuad
     * @param hardwareMap reference to hardwareMap inside your OpMode
     * @param constants odometry setup constants to be sent to the OctoQuad
     *                  NB: this will be ignored if you choose to use
     *                  {@link InitMode#ASSUME_EXTERNAL_INITIALIZATION}
     * @param initMode see {@link InitMode}
     */
    public OctoQuadLocalizer(HardwareMap hardwareMap, OctoQuadConstants constants, InitMode initMode)
    {
        octoQuad = hardwareMap.get(OctoQuad.class, constants.hardwareMapName);

        // If we're not supposed to initialize the hardware, then we're done
        if (initMode == InitMode.ASSUME_EXTERNAL_INITIALIZATION)
        {
            return;
        }

        // Configure a whole bunch of parameters for the absolute localizer
        // --> Read the quick start guide for an explanation of these!!
        // IMPORTANT: these parameter changes will not take effect until
        // the localizer is reset!
        octoQuad.setSingleEncoderDirection(constants.DEADWHEEL_PORT_X, constants.DEADWHEEL_X_DIR);
        octoQuad.setSingleEncoderDirection(constants.DEADWHEEL_PORT_Y, constants.DEADWHEEL_Y_DIR);
        octoQuad.setLocalizerPortX(constants.DEADWHEEL_PORT_X);
        octoQuad.setLocalizerPortY(constants.DEADWHEEL_PORT_Y);
        octoQuad.setLocalizerCountsPerMM_X(constants.X_TICKS_PER_MM);
        octoQuad.setLocalizerCountsPerMM_Y(constants.Y_TICKS_PER_MM);
        octoQuad.setLocalizerTcpOffsetMM_X(constants.TCP_OFFSET_X_MM);
        octoQuad.setLocalizerTcpOffsetMM_Y(constants.TCP_OFFSET_Y_MM);
        octoQuad.setLocalizerImuHeadingScalar(constants.IMU_SCALAR);
        octoQuad.setLocalizerVelocityIntervalMS(constants.VEL_INTVL_MS);
        octoQuad.setI2cRecoveryMode(constants.I2C_RECOVERY_MODE);

        // Resetting the localizer will apply the parameters configured above.
        // This function will NOT block until calibration of the IMU is complete -
        // for that you need to look at the status returned by getLocalizerStatus()
        octoQuad.resetLocalizerAndCalibrateIMU();

        try
        {
            while (octoQuad.getLocalizerStatus() != OctoQuad.LocalizerStatus.RUNNING)
            {
                Thread.sleep(100);
            }
        }
        catch (InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Returns the last cached pose estimate. This does NOT call out
     * to the hardware for updated data, nor does it trigger an
     * external data provider callback, if applicable.
     * @return last cached pose estimate
     */
    @Override
    public Pose getPose()
    {
        return currentPose;
    }

    /**
     * Returns the last cached velocity estimate. This does NOT call out
     * to the hardware for updated data, nor does it trigger an
     * external data provider callback, if applicable.
     * @return last cached velocity estimate
     */
    @Override
    public Pose getVelocity()
    {
        return currentVelocity;
    }

    /**
     * Returns the last cached velocity estimate. This does NOT call out
     * to the hardware for updated data, nor does it trigger an
     * external data provider callback, if applicable.
     * @return last cached velocity estimate
     */
    @Override
    public Vector getVelocityVector()
    {
        return currentVelocity.getAsVector();
    }

    /**
     * Since nobody should be using this after the robot has begun moving,
     * this is functionally the same as setPose(Pose).
     * @param setStart the new current pose estimate
     */
    @Override
    public void setStartPose(Pose setStart)
    {
        setPose(setStart);
    }

    /**
     * Teleport the localizer to a new location (e.g. if you got
     * a ground truth estimate from vision for example)
     * This will send the new pose out to the hardware!
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose)
    {
        octoQuad.setLocalizerPose(
                (int) DistanceUnit.INCH.toMm(setPose.getX()),
                (int) DistanceUnit.INCH.toMm(setPose.getY()),
                (float) AngleUnit.normalizeRadians(setPose.getHeading())
        );

        updateFromHardware();
    }

    protected void updateFromHardware()
    {
        octoQuad.readLocalizerData(localizerData);
        updateInternal();
    }

    protected void updateFromExternalSupplier()
    {
        OctoQuad.LocalizerDataBlock externalLocalizerData = externalDataSupplier.onDataRequest();
        localizerData.localizerStatus = externalLocalizerData.localizerStatus;
        localizerData.crcOk = externalLocalizerData.crcOk;
        localizerData.heading_rad = externalLocalizerData.heading_rad;
        localizerData.posX_mm = externalLocalizerData.posX_mm;
        localizerData.posY_mm = externalLocalizerData.posY_mm;
        localizerData.velX_mmS = externalLocalizerData.velX_mmS;
        localizerData.velY_mmS = externalLocalizerData.velY_mmS;
        localizerData.velHeading_radS = externalLocalizerData.velHeading_radS;
        updateInternal();
    }

    protected void updateInternal()
    {
        if (localizerData.isDataValid())
        {
            currentPose = new Pose(
                    DistanceUnit.MM.toInches(localizerData.posX_mm),
                    DistanceUnit.MM.toInches(localizerData.posY_mm),
                    localizerData.heading_rad
            );

            currentVelocity = new Pose(
                    DistanceUnit.MM.toInches(localizerData.velX_mmS),
                    DistanceUnit.MM.toInches(localizerData.velY_mmS),
                    localizerData.velHeading_radS
            );

            if (localizerData.heading_rad - lastNormalizedHeading > Math.PI / 2)
            {
                headingWraps--;
            }
            else if (localizerData.heading_rad - lastNormalizedHeading < -Math.PI / 2)
            {
                headingWraps++;
            }

            integratedHeading = headingWraps*(2*Math.PI) + localizerData.heading_rad;
            lastNormalizedHeading = localizerData.heading_rad;
        }
    }

    /**
     * Updates the localizer pose estimate, either by contacting the
     * hardware or invoking the external data supplier callback, if
     * one had been registered.
     */
    @Override
    public void update()
    {
        if (externalDataSupplier != null)
        {
            updateFromExternalSupplier();
        }
        else
        {
            updateFromHardware();
        }
    }

    /**
     * Get the un-normalized localizer heading in radians
     * (does not wrap at +/- 2pi)
     * @return un-normalized localizer heading in radians
     */
    @Override
    public double getTotalHeading()
    {
        return integratedHeading;
    }

    /**
     * Do not use.
     * For OctoQuad devices, please use the tuning OpMode provided by DigitalChickenLabs
     * @return throws RuntimeException
     */
    @Override
    public double getForwardMultiplier()
    {
        throw new RuntimeException("For OctoQuad devices, please use the tuning OpMode provided by DigitalChickenLabs");
    }

    /**
     * Do not use.
     * For OctoQuad devices, please use the tuning OpMode provided by DigitalChickenLabs
     * @return throws RuntimeException
     */
    @Override
    public double getLateralMultiplier()
    {
        throw new RuntimeException("For OctoQuad devices, please use the tuning OpMode provided by DigitalChickenLabs");
    }

    /**
     * Do not use.
     * For OctoQuad devices, please use the tuning OpMode provided by DigitalChickenLabs
     * @return throws RuntimeException
     */
    @Override
    public double getTurningMultiplier()
    {
        throw new RuntimeException("For OctoQuad devices, please use the tuning OpMode provided by DigitalChickenLabs");
    }

    /**
     * This recalibrates the IMU and resets the localizer pose to 0,0,0
     */
    @Override
    public void resetIMU()
    {
        octoQuad.resetLocalizerAndCalibrateIMU();

        try
        {
            while (octoQuad.getLocalizerStatus() != OctoQuad.LocalizerStatus.RUNNING)
            {
                Thread.sleep(100);
            }
        }
        catch (InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Get the normalized localizer heading in radians
     * (Wraps at +/- 2pi)
     * @return normalized localizer heading in radians
     */
    @Override
    public double getIMUHeading()
    {
        return currentPose.getHeading();
    }

    /**
     * OctoQuad data is never NaN, because it is guarded by a CRC
     * @return false
     */
    @Override
    public boolean isNAN()
    {
        return false;
    }

    /**
     * Get a handle to the backing OctoQuad object pulled from the hardwareMap
     * @return the backing OctoQuad object pulled from the hardwareMap
     */
    public OctoQuad getOctoQuad()
    {
        return octoQuad;
    }
}
