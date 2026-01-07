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

public class OctoQuadConstants
{
    // #####################################################################################
    // YOU MUST ADJUST THESE CONSTANTS FOR YOUR ROBOT!
    // --> SEE THE OCTOQUAD LOCALIZER QUICKSTART GUIDE
    // #####################################################################################

    /**
     * This creates a new OctoQuadConstants with default values.
     */
    public OctoQuadConstants()
    {
        defaults();
    }

    // SEE THE OCTOQUAD LOCALIZER QUICKSTART GUIDE
    public int DEADWHEEL_PORT_X;
    public int DEADWHEEL_PORT_Y;
    public OctoQuad.EncoderDirection DEADWHEEL_X_DIR;
    public OctoQuad.EncoderDirection DEADWHEEL_Y_DIR;
    public float X_TICKS_PER_MM;
    public float Y_TICKS_PER_MM;
    public float TCP_OFFSET_X_MM;
    public float TCP_OFFSET_Y_MM;
    public float IMU_SCALAR;
    public int VEL_INTVL_MS = 25;
    public String hardwareMapName;
    public OctoQuad.I2cRecoveryMode I2C_RECOVERY_MODE;

    public OctoQuadConstants deadwheelPortX(int port)
    {
        DEADWHEEL_PORT_X = port;
        return this;
    }

    public OctoQuadConstants deadwheelPortY(int port)
    {
        DEADWHEEL_PORT_Y = port;
        return this;
    }

    public OctoQuadConstants deadwheelXDir(OctoQuad.EncoderDirection dir)
    {
        DEADWHEEL_X_DIR = dir;
        return this;
    }

    public OctoQuadConstants deadwheelYDir(OctoQuad.EncoderDirection dir)
    {
        DEADWHEEL_Y_DIR = dir;
        return this;
    }

    public OctoQuadConstants deadwheelXTicksPerMM(float resolution)
    {
        X_TICKS_PER_MM = resolution;
        return this;
    }

    public OctoQuadConstants deadwheelYTicksPerMM(float resolution)
    {
        Y_TICKS_PER_MM = resolution;
        return this;
    }

    public OctoQuadConstants tcpOffsetXMM(float offset)
    {
        TCP_OFFSET_X_MM = offset;
        return this;
    }

    public OctoQuadConstants tcpOffsetYMM(float offset)
    {
        TCP_OFFSET_Y_MM = offset;
        return this;
    }

    public OctoQuadConstants imuScalar(float scale)
    {
        IMU_SCALAR = scale;
        return this;
    }

    public OctoQuadConstants velocityIntervalMs(int interval)
    {
        VEL_INTVL_MS = interval;
        return this;
    }

    public OctoQuadConstants i2cRecoveryMode(OctoQuad.I2cRecoveryMode mode)
    {
        I2C_RECOVERY_MODE = mode;
        return this;
    }

    public OctoQuadConstants name(String name)
    {
        hardwareMapName = name;
        return this;
    }

    public void defaults()
    {
        hardwareMapName = "octoquad";
        DEADWHEEL_PORT_X = 0;
        DEADWHEEL_PORT_Y = 1;
        DEADWHEEL_X_DIR = OctoQuad.EncoderDirection.FORWARD;
        DEADWHEEL_Y_DIR = OctoQuad.EncoderDirection.REVERSE;
        X_TICKS_PER_MM = 12.66f;
        Y_TICKS_PER_MM = 12.66f;
        TCP_OFFSET_X_MM = -97.05f;
        TCP_OFFSET_Y_MM = -156.70f;
        VEL_INTVL_MS = 25;
        IMU_SCALAR = 1.0323f;
        I2C_RECOVERY_MODE = OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR;
    }
}
