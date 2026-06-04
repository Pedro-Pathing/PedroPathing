package com.pedropathing.localization;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * Frozen baseline copy of {@link FusionLocalizer} as of the TreeMap-backed implementation, kept in
 * the test source set purely so we can A/B benchmark it against a future ring-buffer-backed version.
 * <p>
 * This is intentionally <em>not</em> shipped in the production jar and must stay behaviour-identical
 * to the original so the race measures only the data-structure change, not algorithm differences.
 * Do not "improve" this class — it is the control.
 */
public class FusionLocalizerTreeMap implements Localizer {
    /** Floor applied to per-axis measurement variance so a "fully trusted" (variance 0) axis can't
     * freeze that axis or make the innovation covariance S = Pm + R singular. */
    private static final double MEASUREMENT_VARIANCE_FLOOR = 1e-6;
    /** History is kept for this wall-clock window (the acceptable vision-latency budget), making the
     * budget independent of loop rate; {@code bufferSize} additionally caps the entry count. */
    private static final long BUFFER_DURATION_NANOS = 1_000_000_000L;

    private final Localizer deadReckoning;
    private Pose currentPosition;
    private Pose currentVelocity;
    private Pose previousOdometryPose;
    private Matrix P; //State Covariance
    private final Matrix Q; //Process Noise Covariance
    private final Matrix R; //Measurement Noise Covariance
    private final NavigableMap<Long, Pose> poseHistory = new TreeMap<>();
    private final NavigableMap<Long, Pose> odometryHistory = new TreeMap<>();
    private final NavigableMap<Long, Matrix> covarianceHistory = new TreeMap<>();
    private final int bufferSize;

    /**
     * Creates a fusion localizer that corrects a dead-reckoning localizer with vision measurements.
     *
     * @param deadReckoning      the underlying odometry localizer whose increments are fused
     * @param initialCovariance  the initial state covariance diagonal (x, y, heading variances)
     * @param processVariance    the per-axis process-noise coefficients (x, y, heading). <b>Note:</b>
     *                           these are scaled by the distance/rotation actually travelled
     *                           ({@code ΔP = R(θ)·diag(|Δx|·qₓ, |Δy|·q_y, |Δθ|·q_θ)·R(θ)ᵀ}), so the
     *                           units are variance per inch / per radian — not per second². Values
     *                           tuned against an older {@code Q·Δt²} formulation must be re-tuned.
     * @param measurementVariance the default per-axis vision measurement variance (x, y, heading);
     *                           each axis is floored to {@value #MEASUREMENT_VARIANCE_FLOOR}
     * @param bufferSize         the maximum number of history entries to retain (a count cap on top
     *                           of the {@code BUFFER_DURATION_NANOS} wall-clock latency window)
     */
    public FusionLocalizerTreeMap(
            Localizer deadReckoning,
            Pose initialCovariance,
            Pose processVariance,
            Pose measurementVariance,
            int bufferSize
    ) {
        this.deadReckoning = deadReckoning;
        this.currentPosition = new Pose();

        //Standard Deviations for Kalman Filter
        this.P = Matrix.diag(initialCovariance.getX(), initialCovariance.getY(), initialCovariance.getHeading());
        this.Q = Matrix.diag(processVariance.getX(), processVariance.getY(), processVariance.getHeading());
        this.R = Matrix.diag(measurementVariance.getX(), measurementVariance.getY(), measurementVariance.getHeading());
        this.bufferSize = bufferSize;
    }

    /**
     * Source of the monotonic clock (nanoseconds) used to key the history buffers. Exposed so tests
     * can supply deterministic timestamps; production uses {@link System#nanoTime()}.
     *
     * @return the current time in nanoseconds
     */
    protected long currentTimeNanos() {
        return System.nanoTime();
    }

    @Override
    public void update() {
        //Updates odometry
        deadReckoning.update();
        long now = currentTimeNanos();

        Pose odometryPose = deadReckoning.getPose().copy();
        currentVelocity = deadReckoning.getVelocity().copy();

        // Propagate the fused mean by the odometry's body-frame increment composed onto the fused
        // pose: translation follows the *fused* heading, so vision heading corrections are honored,
        // and the SE(2) composition integrates arcs exactly (no forward-Euler drift).
        Pose increment = previousOdometryPose == null
                ? new Pose()
                : relativeTransform(previousOdometryPose, odometryPose);
        P = P.plus(processNoise(increment, currentPosition.getHeading()));
        currentPosition = compose(currentPosition, increment);
        previousOdometryPose = odometryPose;

        poseHistory.put(now, currentPosition.copy());
        odometryHistory.put(now, odometryPose);
        covarianceHistory.put(now, P.copy());
        trim(poseHistory, now);
        trim(odometryHistory, now);
        trim(covarianceHistory, now);
    }

    /**
     * Process-noise contribution G·Q·Gᵀ added to the state covariance for one odometry increment.
     * <p>
     * Instead of the loop-rate-dependent {@code Q·Δt²} of a fixed-rate model, the noise is scaled by
     * the distance/rotation actually travelled:
     * <pre>
     *     ΔP = R(θ) · diag(|Δx|·qₓ, |Δy|·q_y, |Δθ|·q_θ) · R(θ)ᵀ
     * </pre>
     * This matches how odometry drifts, is invariant to loop rate, and keeps a stationary robot's
     * covariance from inflating (|Δ| ≈ 0 ⇒ no growth).
     *
     * @param increment the body-frame odometry increment (Δx, Δy, Δθ)
     * @param heading   the fused heading the increment is applied at, rotating Q into the world frame
     * @return the covariance increment to add to P
     */
    private Matrix processNoise(Pose increment, double heading) {
        Matrix G = Matrix.createRotation(heading);
        Matrix scaledQ = Matrix.diag(
                Math.abs(increment.getX()) * Q.get(0, 0),
                Math.abs(increment.getY()) * Q.get(1, 1),
                Math.abs(increment.getHeading()) * Q.get(2, 2));
        return G.multiply(scaledQ.multiply(G.transposed()));
    }

    /**
     * Adds a vision measurement using the default measurement variance
     * @param measuredPose the measured position by the camera, enter NaN to a specific axis if the camera couldn't measure that axis
     * @param timestamp the timestamp of the measurement
     */
    public void addMeasurement(Pose measuredPose, long timestamp) {
        addMeasurement(measuredPose, timestamp, null);
    }

    /**
     * Adds a vision measurement with a custom variance for this specific measurement
     * @param measuredPose the measured position by the camera, enter NaN to a specific axis if the camera couldn't measure that axis
     * @param timestamp the timestamp of the measurement
     * @param measurementVariance the variance for this specific measurement (x, y, heading), or null to use the default
     */
    public void addMeasurement(Pose measuredPose, long timestamp, Pose measurementVariance) {
        Matrix measurementR = measurementVariance == null
                ? R.copy()
                : Matrix.diag(measurementVariance.getX(), measurementVariance.getY(), measurementVariance.getHeading());
        // Floor variances so a "fully trusted" axis (variance 0) can't freeze the axis or make S singular.
        for (int i = 0; i < 3; i++)
            measurementR.set(i, i, Math.max(measurementR.get(i, i), MEASUREMENT_VARIANCE_FLOOR));

        // Reject if timestamp is outside our poseHistory time window
        if (poseHistory.isEmpty() || timestamp < poseHistory.firstKey() || timestamp > poseHistory.lastKey())
            return;

        Pose pastPose = interpolate(timestamp, poseHistory);
        if (pastPose == null)
            pastPose = getPose();

        // Measurement residual y = z - x
        boolean measX = !Double.isNaN(measuredPose.getX());
        boolean measY = !Double.isNaN(measuredPose.getY());
        boolean measH = !Double.isNaN(measuredPose.getHeading());

        Matrix y = new Matrix(new double[][]{
                {measX ? measuredPose.getX() - pastPose.getX() : 0},
                {measY ? measuredPose.getY() - pastPose.getY() : 0},
                {measH ? MathFunctions.normalizeAngleSigned(measuredPose.getHeading() - pastPose.getHeading()) : 0}
        });

        // Measurement mask M
        Matrix M = Matrix.diag(
                measX ? 1 : 0,
                measY ? 1 : 0,
                measH ? 1 : 0
        );

        // Covariance at measurement time
        Matrix Pm = covarianceHistory.floorEntry(timestamp).getValue();

        // Innovation covariance S = P + R
        Matrix S = Pm.plus(measurementR);

        // Apply gain K = P * (P + R)^(-1); skip (don't crash) if S is singular / ill-conditioned
        Matrix K;
        try {
            K = Pm.multiply(S.inverse());
        } catch (IllegalArgumentException | IllegalStateException e) {
            return;
        }

        // Apply mask
        K = M.multiply(K);
        y = M.multiply(y);

        // State update
        Matrix Ky = K.multiply(y);
        Pose updatedPast = new Pose(
                pastPose.getX() + Ky.get(0, 0),
                pastPose.getY() + Ky.get(1, 0),
                MathFunctions.normalizeAngle(pastPose.getHeading() + Ky.get(2, 0))
        );
        poseHistory.put(timestamp, updatedPast);

        // Joseph-form covariance update
        Matrix I = Matrix.identity(3);
        Matrix IK = I.minus(K);
        Matrix updatedCovariance =
                IK.multiply(Pm).multiply(IK.transposed())
                        .plus(K.multiply(measurementR).multiply(K.transposed()));

        covarianceHistory.put(timestamp, updatedCovariance);

        // Forward propagate pose + covariance from the correction, using the odometry's relative
        // transforms (same SE(2) composition as update(), so the correction's heading is honored).
        Pose prevPose = updatedPast;
        Pose prevOdom = interpolate(timestamp, odometryHistory);
        Matrix prevCov = updatedCovariance;

        for (Long t : poseHistory.tailMap(timestamp, false).keySet()) {
            Pose currOdom = interpolate(t, odometryHistory);
            Pose increment = (prevOdom == null || currOdom == null)
                    ? new Pose()
                    : relativeTransform(prevOdom, currOdom);

            Pose nextPose = compose(prevPose, increment);
            poseHistory.put(t, nextPose);

            prevCov = prevCov.plus(processNoise(increment, prevPose.getHeading()));
            covarianceHistory.put(t, prevCov);

            prevPose = nextPose;
            prevOdom = currOdom;
        }

        currentPosition = poseHistory.lastEntry().getValue().copy();
        P = covarianceHistory.lastEntry().getValue().copy();
    }

    //Performs linear interpolation inside the history map for the value at a given timestamp
    private static Pose interpolate(long timestamp, NavigableMap<Long, Pose> history) {
        Long lowerKey = history.floorKey(timestamp);
        Long upperKey = history.ceilingKey(timestamp);

        if (lowerKey == null || upperKey == null) return null;
        if (lowerKey.equals(upperKey)) return history.get(lowerKey).copy();

        Pose lowerPose = history.get(lowerKey);
        Pose upperPose = history.get(upperKey);

        double ratio = (double) (timestamp - lowerKey) / (upperKey - lowerKey);

        double x = lowerPose.getX() + ratio * (upperPose.getX() - lowerPose.getX());
        double y = lowerPose.getY() + ratio * (upperPose.getY() - lowerPose.getY());
        double headingDiff = MathFunctions.getSmallestAngleDifference(upperPose.getHeading(), lowerPose.getHeading());
        double heading = MathFunctions.normalizeAngle(lowerPose.getHeading() + ratio * headingDiff);

        return new Pose(x, y, heading);
    }

    /** SE(2) body-frame increment that takes {@code from} to {@code to}: from⁻¹ ⊕ to. */
    private static Pose relativeTransform(Pose from, Pose to) {
        double cos = Math.cos(from.getHeading());
        double sin = Math.sin(from.getHeading());
        double dx = to.getX() - from.getX();
        double dy = to.getY() - from.getY();
        return new Pose(
                dx * cos + dy * sin,
                -dx * sin + dy * cos,
                MathFunctions.normalizeAngleSigned(to.getHeading() - from.getHeading())
        );
    }

    /** SE(2) composition {@code base ⊕ relative}: applies a body-frame increment at base's heading. */
    private static Pose compose(Pose base, Pose relative) {
        double cos = Math.cos(base.getHeading());
        double sin = Math.sin(base.getHeading());
        return new Pose(
                base.getX() + relative.getX() * cos - relative.getY() * sin,
                base.getY() + relative.getX() * sin + relative.getY() * cos,
                MathFunctions.normalizeAngle(base.getHeading() + relative.getHeading())
        );
    }

    /** Drops history older than the latency window, then caps total entries at {@code bufferSize}. */
    private void trim(NavigableMap<Long, ?> history, long now) {
        Long floor = history.floorKey(now - BUFFER_DURATION_NANOS);
        if (floor != null) history.headMap(floor, false).clear();
        while (history.size() > bufferSize) history.pollFirstEntry();
    }

    @Override
    public Pose getPose() { return currentPosition; }

    @Override
    public Pose getVelocity() {
        return currentVelocity != null ? currentVelocity : deadReckoning.getVelocity();
    }

    @Override
    public Vector getVelocityVector() { return getVelocity().getAsVector(); }

    @Override
    public void setStartPose(Pose setStart) {
        deadReckoning.setStartPose(setStart);
        previousOdometryPose = deadReckoning.getPose().copy();
        poseHistory.put(0L, setStart.copy());
        odometryHistory.put(0L, previousOdometryPose.copy());
        covarianceHistory.put(0L, P.copy());
        currentPosition = setStart.copy();
    }

    @Override
    public void setPose(Pose setPose) {
        currentPosition = setPose.copy();
        deadReckoning.setPose(setPose);
        previousOdometryPose = deadReckoning.getPose().copy();

        // NavigableMap.lastEntry() returns an immutable snapshot, so overwrite via put(lastKey, ...).
        if (poseHistory.isEmpty()) {
            setStartPose(setPose);
        } else {
            poseHistory.put(poseHistory.lastKey(), setPose.copy());
            if (!odometryHistory.isEmpty())
                odometryHistory.put(odometryHistory.lastKey(), previousOdometryPose.copy());
        }
    }

    @Override
    public double getTotalHeading() { return currentPosition.getHeading(); }

    @Override
    public double getForwardMultiplier() { return deadReckoning.getForwardMultiplier(); }

    @Override
    public double getLateralMultiplier() { return deadReckoning.getLateralMultiplier(); }

    @Override
    public double getTurningMultiplier() { return deadReckoning.getTurningMultiplier(); }

    @Override
    public void resetIMU() throws InterruptedException { deadReckoning.resetIMU(); }

    @Override
    public double getIMUHeading() { return deadReckoning.getIMUHeading(); }

    @Override
    public boolean isNAN() {
        return Double.isNaN(currentPosition.getX()) || Double.isNaN(currentPosition.getY()) || Double.isNaN(currentPosition.getHeading());
    }

    @Override
    public double getAngularVelocity() {
        return deadReckoning.getAngularVelocity();
    }
}
