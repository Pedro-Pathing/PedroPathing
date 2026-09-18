/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.localization;

/**
 * Supplies a single encoder position in raw ticks.
 *
 * <p>Values are hardware-sign ticks. Pedro applies its configured encoder multiplier separately.
 * Implementations may read a motor, a simulator, a replay, or a cycle-level cache.
 */
@FunctionalInterface
public interface EncoderPositionSource {
    /**
     * Current encoder position in raw ticks.
     *
     * @return encoder ticks, including the hardware sign and excluding Pedro's multiplier
     */
    int positionTicks();
}
