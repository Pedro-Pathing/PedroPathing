package com.pedropathing.revhub.localizers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.junit.Test;

public class EncoderSourceTest {
    private static final double EPS = 1e-9;

    @Test
    public void motorBackedEncoderTracksDeltaAndPhysicallyResets() {
        FakeDcMotorEx fake = new FakeDcMotorEx();
        Encoder encoder = new Encoder(fake.motor());
        assertTrue(fake.stopAndResetCount >= 1);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, fake.mode);

        int readsAfterConstruction = fake.getCurrentPositionCount;
        fake.position = 40;
        encoder.update();
        assertEquals(1, fake.getCurrentPositionCount - readsAfterConstruction);
        assertEquals(40.0, encoder.getDeltaPosition(), EPS);

        fake.position = 55;
        encoder.update();
        assertEquals(15.0, encoder.getDeltaPosition(), EPS);

        int resets = fake.stopAndResetCount;
        encoder.reset();
        assertEquals(resets + 1, fake.stopAndResetCount);
        assertEquals(0, fake.position);
    }

    @Test
    public void injectedSourceProducesTheSameDeltasAsMotorBacked() {
        FakeDcMotorEx fake = new FakeDcMotorEx();
        Encoder motorEncoder = new Encoder(fake.motor());

        CountingIntSupplier source = new CountingIntSupplier(0);
        Encoder injected = Encoder.from(source);

        fake.position = 25;
        source.value = 25;
        motorEncoder.update();
        injected.update();
        assertEquals(motorEncoder.getDeltaPosition(), injected.getDeltaPosition(), EPS);

        fake.position = 10;
        source.value = 10;
        motorEncoder.update();
        injected.update();
        assertEquals(motorEncoder.getDeltaPosition(), injected.getDeltaPosition(), EPS);
    }

    @Test
    public void injectedUpdateReadsTheSourceOnce() {
        CountingIntSupplier source = new CountingIntSupplier(0);
        Encoder encoder = Encoder.from(source);
        source.calls = 0;

        source.value = 12;
        encoder.update();
        assertEquals(1, source.calls);
        encoder.getDeltaPosition();
        encoder.getDeltaPosition();
        encoder.getMultiplier();
        assertEquals(1, source.calls);
    }

    @Test
    public void injectedResetRebasesWithoutAPhysicalMotor() {
        CountingIntSupplier source = new CountingIntSupplier(100);
        Encoder encoder = Encoder.from(source);
        source.value = 130;
        encoder.update();
        assertEquals(30.0, encoder.getDeltaPosition(), EPS);

        source.calls = 0;
        encoder.reset();
        assertEquals(1, source.calls);

        source.calls = 0;
        encoder.update();
        assertEquals(1, source.calls);
        assertEquals(0.0, encoder.getDeltaPosition(), EPS);

        source.value = 150;
        encoder.update();
        assertEquals(20.0, encoder.getDeltaPosition(), EPS);
    }

    @Test
    public void motorDirectionAndPedroMultiplierBothApplyOnHardwarePath() {
        FakeDcMotorEx fake = new FakeDcMotorEx();
        Encoder encoder = new Encoder(fake.motor());
        fake.position = 8;
        encoder.update();
        assertEquals(8.0, encoder.getDeltaPosition(), EPS);

        encoder.setDirection(Encoder.REVERSE);
        fake.position = 18;
        encoder.update();
        assertEquals(-10.0, encoder.getDeltaPosition(), EPS);

        fake.direction = DcMotorSimple.Direction.REVERSE;
        fake.position = 20;
        encoder.update();
        assertEquals(2.0, encoder.getDeltaPosition(), EPS);
    }

    @Test
    public void injectedDirectionUsesOnlyPedroMultiplier() {
        CountingIntSupplier source = new CountingIntSupplier(0);
        Encoder encoder = Encoder.from(source);
        source.value = 8;
        encoder.update();
        assertEquals(8.0, encoder.getDeltaPosition(), EPS);

        encoder.setDirection(Encoder.REVERSE);
        source.value = 18;
        encoder.update();
        assertEquals(-10.0, encoder.getDeltaPosition(), EPS);
    }
}
