package com.pedropathing.revhub.localizers;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OctoQuadConfig {
    public final ConfigVar<String> name = ConfigVar.required();
    public final ConfigVar<OctoQuad.EncoderDirection> xPodDirection = ConfigVar.required();
    public final ConfigVar<OctoQuad.EncoderDirection> yPodDirection = ConfigVar.required();
    public final ConfigVar<Integer> xPodPort = ConfigVar.of(0);
    public final ConfigVar<Integer> yPodPort = ConfigVar.of(1);
    public final ConfigVar<Float> xPodOffset = ConfigVar.required();
    public final ConfigVar<Float> yPodOffset = ConfigVar.required();

    /** Distance unit used for xPodOffset and yPodOffset **/
    public final ConfigVar<DistanceUnit> offsetUnits = ConfigVar.of(DistanceUnit.INCH);

    /** Global distance unit for all poses and positions */
    public final ConfigVar<DistanceUnit> globalDistanceUnit = ConfigVar.of(DistanceUnit.INCH);

    /** Distance unit used for ticksPerUnit **/
    public final ConfigVar<DistanceUnit> encoderResolutionUnit = ConfigVar.of(DistanceUnit.INCH);

    public final ConfigVar<Double> ticksPerUnit = ConfigVar.required();

    /**
     * Use this to make the imu much more accurate.
     * For example, turn your robot 10 times (3600 degrees) and measure how far it is off.
     * Then put your new value as imuScalar = measuredRotation/realRotation.
     * So if your robot reads 50 extra degrees, then your imuScalar would be 3650/3600 ~ 1.014
     **/
    public final ConfigVar<Float> headingScalar = ConfigVar.of(1.0f);

    public final ConfigVar<OctoQuad.I2cRecoveryMode> i2cRecoveryMode = ConfigVar.of(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);

    /**
     * Set the period of translational velocity calculation.
     * Longer periods give higher resolution with more latency, shorter periods give lower resolution with less latency.
     */
    public final ConfigVar<Integer> localizerVelocityIntervalMS = ConfigVar.of(25);

    public OctoQuadConfig(Configuration<OctoQuadConfig> config) {
        config.configure(this);
    }
}