package com.pedropathing.revhub.localizers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

public class RevLocalizerSourceInjectionTest {
    private static final double EPS = 1e-6;

    @Test
    public void threeWheelInjectedSourcesAreReadOncePerUpdate() {
        CountingIntSupplier left = new CountingIntSupplier();
        CountingIntSupplier right = new CountingIntSupplier();
        CountingIntSupplier strafe = new CountingIntSupplier();
        ThreeWheelLocalizer localizer = new ThreeWheelLocalizer(threeWheelConfig(), left, right, strafe);
        zero(left, right, strafe);

        left.value = 10;
        right.value = 10;
        localizer.update();
        assertEquals(1, left.calls);
        assertEquals(1, right.calls);
        assertEquals(1, strafe.calls);
        assertEquals(10.0, localizer.pose().x(), EPS);
        assertEquals(0.0, localizer.pose().y(), EPS);
        assertEquals(0.0, localizer.pose().heading(), EPS);

        queryState(localizer);
        assertEquals(1, left.calls);
        assertEquals(1, right.calls);
        assertEquals(1, strafe.calls);

        Pose first = localizer.pose();
        Pose second = localizer.pose();
        assertEquals(first.x(), second.x(), EPS);
        assertEquals(first.y(), second.y(), EPS);
        assertEquals(first.heading(), second.heading(), EPS);
    }

    @Test
    public void twoWheelInjectedSourcesAreReadOncePerUpdate() {
        CountingIntSupplier xPod = new CountingIntSupplier();
        CountingIntSupplier yPod = new CountingIntSupplier();
        CountingDoubleSupplier heading = new CountingDoubleSupplier();
        TwoWheelLocalizer localizer = new TwoWheelLocalizer(twoWheelConfig(), xPod, yPod, heading);
        zero(xPod, yPod);
        heading.calls = 0;

        xPod.value = 10;
        localizer.update();
        assertEquals(1, xPod.calls);
        assertEquals(1, yPod.calls);
        assertEquals(1, heading.calls);
        assertEquals(10.0, localizer.pose().x(), EPS);

        queryState(localizer);
        assertEquals(1, xPod.calls);
        assertEquals(1, yPod.calls);
        assertEquals(1, heading.calls);
    }

    @Test
    public void threeWheelImuInjectedSourcesAreReadOncePerUpdate() {
        boolean previousUseImu = ThreeWheelIMULocalizer.useIMU;
        ThreeWheelIMULocalizer.useIMU = true;
        try {
            CountingIntSupplier left = new CountingIntSupplier();
            CountingIntSupplier right = new CountingIntSupplier();
            CountingIntSupplier strafe = new CountingIntSupplier();
            CountingDoubleSupplier heading = new CountingDoubleSupplier();
            ThreeWheelIMULocalizer localizer = new ThreeWheelIMULocalizer(
                    threeWheelImuConfig(), left, right, strafe, heading);
            zero(left, right, strafe);
            heading.calls = 0;

            heading.value = 0.25;
            localizer.update();
            assertEquals(1, left.calls);
            assertEquals(1, right.calls);
            assertEquals(1, strafe.calls);
            assertEquals(1, heading.calls);
            assertEquals(0.25, localizer.pose().heading(), EPS);

            queryState(localizer);
            assertEquals(1, heading.calls);
        } finally {
            ThreeWheelIMULocalizer.useIMU = previousUseImu;
        }
    }

    @Test
    public void injectedResetRebasesWithoutImplyingAPhysicalEncoderReset() {
        CountingIntSupplier left = new CountingIntSupplier();
        CountingIntSupplier right = new CountingIntSupplier();
        CountingIntSupplier strafe = new CountingIntSupplier();
        ThreeWheelLocalizer localizer = new ThreeWheelLocalizer(threeWheelConfig(), left, right, strafe);

        left.value = 10;
        right.value = 10;
        localizer.update();
        assertEquals(10.0, localizer.pose().x(), EPS);

        zero(left, right, strafe);
        localizer.setPose(new Pose(1, 2, 0.1));
        assertEquals(1, left.calls);
        assertEquals(1, right.calls);
        assertEquals(1, strafe.calls);
        assertEquals(1.0, localizer.pose().x(), EPS);
        assertEquals(2.0, localizer.pose().y(), EPS);

        zero(left, right, strafe);
        localizer.update();
        assertEquals(1, left.calls);
        assertEquals(1.0, localizer.pose().x(), EPS);
    }

    @Test
    public void injectedPedroMultiplierReversesDelta() {
        CountingIntSupplier left = new CountingIntSupplier();
        CountingIntSupplier right = new CountingIntSupplier();
        CountingIntSupplier strafe = new CountingIntSupplier();
        ThreeWheelConfig config = new ThreeWheelConfig(c -> {
            c.leftPodY.set(1.0);
            c.rightPodY.set(-1.0);
            c.strafePodX.set(0.0);
            c.forwardTicksToInches.set(1.0);
            c.strafeTicksToInches.set(1.0);
            c.turnTicksToRadians.set(1.0);
            c.leftEncoderDirection.set(Encoder.REVERSE);
            c.rightEncoderDirection.set(Encoder.REVERSE);
            c.strafeEncoderDirection.set(Encoder.FORWARD);
        });
        ThreeWheelLocalizer localizer = new ThreeWheelLocalizer(config, left, right, strafe);
        left.value = 10;
        right.value = 10;
        localizer.update();
        assertEquals(-10.0, localizer.pose().x(), EPS);
    }

    @Test
    public void legacyHardwareMapConstructorsRemainOnThePublicApi() throws Exception {
        assertNotNull(ThreeWheelLocalizer.class.getConstructor(HardwareMap.class, ThreeWheelConfig.class));
        assertNotNull(TwoWheelLocalizer.class.getConstructor(HardwareMap.class, TwoWheelConfig.class));
        assertNotNull(ThreeWheelIMULocalizer.class.getConstructor(
                HardwareMap.class, ThreeWheelIMUConfig.class));
        assertNotNull(Encoder.class.getConstructor(com.qualcomm.robotcore.hardware.DcMotorEx.class));
    }

    @Test
    public void injectedSourcesConstructWithMethodReferences() {
        class ExternalSnapshot {
            int leftTicks;
            int rightTicks;
            int strafeTicks;

            int leftTicks() {
                return leftTicks;
            }

            int rightTicks() {
                return rightTicks;
            }

            int strafeTicks() {
                return strafeTicks;
            }
        }

        ExternalSnapshot externalSnapshot = new ExternalSnapshot();
        Localizer localizer = new ThreeWheelLocalizer(
                threeWheelConfig(),
                externalSnapshot::leftTicks,
                externalSnapshot::rightTicks,
                externalSnapshot::strafeTicks);
        assertNotNull(localizer.pose());
        assertNotNull(localizer.state());
    }

    private static void queryState(Localizer localizer) {
        localizer.pose();
        localizer.velocity();
        localizer.twist();
        localizer.state();
        localizer.debug();
    }

    private static void zero(CountingIntSupplier... sources) {
        for (CountingIntSupplier source : sources) {
            source.calls = 0;
        }
    }

    private static ThreeWheelConfig threeWheelConfig() {
        return new ThreeWheelConfig(c -> {
            c.leftPodY.set(1.0);
            c.rightPodY.set(-1.0);
            c.strafePodX.set(0.0);
            c.forwardTicksToInches.set(1.0);
            c.strafeTicksToInches.set(1.0);
            c.turnTicksToRadians.set(1.0);
            c.leftEncoderDirection.set(Encoder.FORWARD);
            c.rightEncoderDirection.set(Encoder.FORWARD);
            c.strafeEncoderDirection.set(Encoder.FORWARD);
        });
    }

    private static TwoWheelConfig twoWheelConfig() {
        return new TwoWheelConfig(c -> {
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.forwardTicksToInches.set(1.0);
            c.strafeTicksToInches.set(1.0);
            c.xPodDirection.set(Encoder.FORWARD);
            c.yPodDirection.set(Encoder.FORWARD);
        });
    }

    private static ThreeWheelIMUConfig threeWheelImuConfig() {
        return new ThreeWheelIMUConfig(c -> {
            c.leftPodY.set(1.0);
            c.rightPodY.set(-1.0);
            c.strafePodX.set(0.0);
            c.forwardTicksToInches.set(1.0);
            c.strafeTicksToInches.set(1.0);
            c.turnTicksToRadians.set(1.0);
            c.leftEncoderDirection.set(Encoder.FORWARD);
            c.rightEncoderDirection.set(Encoder.FORWARD);
            c.strafeEncoderDirection.set(Encoder.FORWARD);
        });
    }
}
