package com.pedropathing.localization;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

public class FusionLocalizer implements Localizer {
    /** Floor applied to per-axis measurement variance so a "fully trusted" (variance 0) axis can't
     * freeze that axis or make the innovation covariance S = Pm + R singular. */
    private static final double MEASUREMENT_VARIANCE_FLOOR = 1e-6;
    /** History is kept for this wall-clock window (the acceptable vision-latency budget), making the
     * budget independent of loop rate; {@code bufferSize} additionally caps the entry count. */
    private static final long BUFFER_DURATION_NANOS = 1_000_000_000L;

    private final Localizer deadReckoning;

    // Fused state, previous odometry, and velocity are held as raw components rather than Pose
    // objects so the per-update hot path allocates nothing; an immutable Pose is materialized only
    // when handed out via getPose()/getVelocity().
    private double curX, curY, curH;
    private double velX, velY, velH;
    private boolean hasVelocity;
    private double prevOdomX, prevOdomY, prevOdomH;
    private boolean hasPrevOdom;

    private Matrix P; //State Covariance
    private final Matrix Q; //Process Noise Covariance
    private final Matrix R; //Measurement Noise Covariance
    private final History history;
    private final int bufferSize;

    /** Reusable [x, y, heading] output buffer for the SE(2) helpers and history interpolation reads. */
    private final double[] pose = new double[3];

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
    public FusionLocalizer(
            Localizer deadReckoning,
            Pose initialCovariance,
            Pose processVariance,
            Pose measurementVariance,
            int bufferSize
    ) {
        this.deadReckoning = deadReckoning;

        // Kalman filter covariances (diagonal)
        this.P = Matrix.diag(initialCovariance.getX(), initialCovariance.getY(), initialCovariance.getHeading());
        this.Q = Matrix.diag(processVariance.getX(), processVariance.getY(), processVariance.getHeading());
        this.R = Matrix.diag(measurementVariance.getX(), measurementVariance.getY(), measurementVariance.getHeading());
        this.bufferSize = bufferSize;
        this.history = new History(bufferSize);
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
        deadReckoning.update();
        long now = currentTimeNanos();

        Pose odometryPose = deadReckoning.getPose();
        double ox = odometryPose.getX(), oy = odometryPose.getY(), oh = odometryPose.getHeading();
        Pose velocity = deadReckoning.getVelocity();
        velX = velocity.getX(); velY = velocity.getY(); velH = velocity.getHeading();
        hasVelocity = true;

        // Body-frame increment of the odometry since the previous sample (zero on the first update).
        double incX = 0, incY = 0, incH = 0;
        if (hasPrevOdom) {
            relativeTransform(prevOdomX, prevOdomY, prevOdomH, ox, oy, oh, pose);
            incX = pose[0]; incY = pose[1]; incH = pose[2];
        }

        // Grow the covariance, then compose the increment onto the fused pose. Translation follows
        // the *fused* heading, so vision heading corrections are honored and the step is exact in SE(2).
        addProcessNoiseInPlace(P, incX, incY, incH, curH);
        composeOnto(curX, curY, curH, incX, incY, incH, pose);
        curX = pose[0]; curY = pose[1]; curH = pose[2];

        prevOdomX = ox; prevOdomY = oy; prevOdomH = oh;
        hasPrevOdom = true;

        history.append(now, curX, curY, curH, ox, oy, oh, P.copy());
        history.trimOlderThan(now - BUFFER_DURATION_NANOS);
        history.enforceCountCap(bufferSize);
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

        // Reject if timestamp is outside our history time window
        if (history.isEmpty() || timestamp < history.firstTime() || timestamp > history.lastTime())
            return;

        // Fused pose at the measurement time
        double pastX, pastY, pastH;
        if (history.interpolateFusedInto(timestamp, pose)) {
            pastX = pose[0]; pastY = pose[1]; pastH = pose[2];
        } else {
            pastX = curX; pastY = curY; pastH = curH;
        }

        // Measurement residual y = z - x
        boolean measX = !Double.isNaN(measuredPose.getX());
        boolean measY = !Double.isNaN(measuredPose.getY());
        boolean measH = !Double.isNaN(measuredPose.getHeading());

        Matrix y = new Matrix(new double[][]{
                {measX ? measuredPose.getX() - pastX : 0},
                {measY ? measuredPose.getY() - pastY : 0},
                {measH ? MathFunctions.normalizeAngleSigned(measuredPose.getHeading() - pastH) : 0}
        });

        // Measurement mask M
        Matrix M = Matrix.diag(
                measX ? 1 : 0,
                measY ? 1 : 0,
                measH ? 1 : 0
        );

        // Covariance at measurement time (floor entry: latest sample at or before the timestamp)
        Matrix Pm = history.covAt(history.floorIndex(timestamp));

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
        double updX = pastX + Ky.get(0, 0);
        double updY = pastY + Ky.get(1, 0);
        double updH = MathFunctions.normalizeAngle(pastH + Ky.get(2, 0));

        // Joseph-form covariance update
        Matrix I = Matrix.identity(3);
        Matrix IK = I.minus(K);
        Matrix updatedCovariance =
                IK.multiply(Pm).multiply(IK.transposed())
                        .plus(K.multiply(measurementR).multiply(K.transposed()));

        // Insert (or overwrite) the corrected sample at the measurement time. The odometry pose at
        // that time is interpolated so every sample carries a full (fused, odom, covariance) row.
        boolean haveOdom = history.interpolateOdomInto(timestamp, pose);
        double odomX = haveOdom ? pose[0] : 0;
        double odomY = haveOdom ? pose[1] : 0;
        double odomH = haveOdom ? pose[2] : 0;
        history.putCorrection(timestamp, updX, updY, updH, odomX, odomY, odomH, updatedCovariance);

        // Re-propagate every later sample from the correction, replaying the stored odometry
        // increments (same SE(2) composition as update(), so the correction's heading is honored).
        double prevX = updX, prevY = updY, prevH = updH;
        double prevOX = odomX, prevOY = odomY, prevOH = odomH;
        boolean prevHaveOdom = haveOdom;
        Matrix prevCov = updatedCovariance;

        for (int i = history.floorIndex(timestamp) + 1; i < history.size(); i++) {
            double currOX = history.odomXAt(i), currOY = history.odomYAt(i), currOH = history.odomHAt(i);

            double incX = 0, incY = 0, incH = 0;
            if (prevHaveOdom) {
                relativeTransform(prevOX, prevOY, prevOH, currOX, currOY, currOH, pose);
                incX = pose[0]; incY = pose[1]; incH = pose[2];
            }

            composeOnto(prevX, prevY, prevH, incX, incY, incH, pose);
            double nextX = pose[0], nextY = pose[1], nextH = pose[2];
            history.setFused(i, nextX, nextY, nextH);

            // Copy once (each stored row needs its own matrix), then accumulate noise in place.
            Matrix nextCov = prevCov.copy();
            addProcessNoiseInPlace(nextCov, incX, incY, incH, prevH);
            history.setCov(i, nextCov);

            prevX = nextX; prevY = nextY; prevH = nextH;
            prevOX = currOX; prevOY = currOY; prevOH = currOH; prevHaveOdom = true;
            prevCov = nextCov;
        }

        curX = history.lastFusedX();
        curY = history.lastFusedY();
        curH = history.lastFusedH();
        P = history.lastCov().copy();

        // A new sample was inserted at the correction time; re-apply the count cap (drops the oldest)
        // so the buffer is back at <= bufferSize before the next measurement or update.
        history.enforceCountCap(bufferSize);
    }

    /**
     * Writes the SE(2) body-frame increment that takes pose {@code from} to pose {@code to}
     * (from⁻¹ ⊕ to) into {@code out} as [Δx, Δy, Δθ].
     */
    private static void relativeTransform(double fromX, double fromY, double fromH,
                                          double toX, double toY, double toH, double[] out) {
        double co = Math.cos(fromH), si = Math.sin(fromH);
        double dx = toX - fromX, dy = toY - fromY;
        out[0] = dx * co + dy * si;
        out[1] = -dx * si + dy * co;
        out[2] = MathFunctions.normalizeAngleSigned(toH - fromH);
    }

    /**
     * Writes the SE(2) composition {@code base ⊕ increment} into {@code out} as [x, y, heading]:
     * applies a body-frame increment at the base pose's heading.
     */
    private static void composeOnto(double baseX, double baseY, double baseH,
                                    double incX, double incY, double incH, double[] out) {
        double co = Math.cos(baseH), si = Math.sin(baseH);
        out[0] = baseX + incX * co - incY * si;
        out[1] = baseY + incX * si + incY * co;
        out[2] = MathFunctions.normalizeAngle(baseH + incH);
    }

    /**
     * Adds the process-noise contribution G·Q·Gᵀ for one odometry increment directly into
     * {@code target} (a 3×3 covariance), allocating nothing.
     * <p>
     * Instead of the loop-rate-dependent {@code Q·Δt²} of a fixed-rate model, the noise is scaled by
     * the distance/rotation actually travelled:
     * <pre>
     *     ΔP = R(θ) · diag(|Δx|·qₓ, |Δy|·q_y, |Δθ|·q_θ) · R(θ)ᵀ
     * </pre>
     * This matches how odometry drifts, is invariant to loop rate, and keeps a stationary robot's
     * covariance from inflating (|Δ| ≈ 0 ⇒ no growth).
     * <p>
     * Because G = R(θ) is a planar rotation (identity on the heading axis) and Q is diagonal, the
     * product has a closed form, so this writes the five affected entries in place rather than
     * allocating the rotation, the diagonal, and two matrix-multiply temporaries each call. The
     * factor order matches {@code G.multiply(scaledQ.multiply(G.transposed()))} entry-for-entry.
     *
     * @param target  the covariance to accumulate into (mutated)
     * @param ix      body-frame increment Δx
     * @param iy      body-frame increment Δy
     * @param ih      body-frame increment Δθ
     * @param heading the fused heading the increment is applied at, rotating Q into the world frame
     */
    private void addProcessNoiseInPlace(Matrix target, double ix, double iy, double ih, double heading) {
        double co = Math.cos(heading);
        double si = Math.sin(heading);
        double qx = Math.abs(ix) * Q.get(0, 0);
        double qy = Math.abs(iy) * Q.get(1, 1);
        double qh = Math.abs(ih) * Q.get(2, 2);

        // scaledQ · Gᵀ = [[qx·co, qx·si, 0], [-qy·si, qy·co, 0], [0, 0, qh]]
        double m00 = qx * co, m01 = qx * si;
        double m10 = -(qy * si), m11 = qy * co;

        // G · (scaledQ · Gᵀ), top-left 2×2 (heading axis is just qh)
        double d00 = co * m00 + (-si) * m10;
        double d01 = co * m01 + (-si) * m11;
        double d10 = si * m00 + co * m10;
        double d11 = si * m01 + co * m11;

        target.set(0, 0, target.get(0, 0) + d00);
        target.set(0, 1, target.get(0, 1) + d01);
        target.set(1, 0, target.get(1, 0) + d10);
        target.set(1, 1, target.get(1, 1) + d11);
        target.set(2, 2, target.get(2, 2) + qh);
    }

    @Override
    public Pose getPose() { return new Pose(curX, curY, curH); }

    @Override
    public Pose getVelocity() {
        return hasVelocity ? new Pose(velX, velY, velH) : deadReckoning.getVelocity();
    }

    @Override
    public Vector getVelocityVector() { return getVelocity().getAsVector(); }

    @Override
    public void setStartPose(Pose setStart) {
        deadReckoning.setStartPose(setStart);
        Pose odometryPose = deadReckoning.getPose();
        prevOdomX = odometryPose.getX(); prevOdomY = odometryPose.getY(); prevOdomH = odometryPose.getHeading();
        hasPrevOdom = true;
        curX = setStart.getX(); curY = setStart.getY(); curH = setStart.getHeading();
        history.put(0L, curX, curY, curH, prevOdomX, prevOdomY, prevOdomH, P.copy());
    }

    @Override
    public void setPose(Pose setPose) {
        curX = setPose.getX(); curY = setPose.getY(); curH = setPose.getHeading();
        deadReckoning.setPose(setPose);
        Pose odometryPose = deadReckoning.getPose();
        prevOdomX = odometryPose.getX(); prevOdomY = odometryPose.getY(); prevOdomH = odometryPose.getHeading();
        hasPrevOdom = true;

        if (history.isEmpty()) {
            setStartPose(setPose);
        } else {
            int last = history.size() - 1;
            history.setFused(last, curX, curY, curH);
            history.setOdom(last, prevOdomX, prevOdomY, prevOdomH);
        }
    }

    @Override
    public double getTotalHeading() { return curH; }

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
        return Double.isNaN(curX) || Double.isNaN(curY) || Double.isNaN(curH);
    }

    @Override
    public double getAngularVelocity() {
        return deadReckoning.getAngularVelocity();
    }

    /**
     * Fixed-capacity, time-sorted ring buffer holding the (timestamp, fused pose, odometry pose,
     * covariance) history. Replaces three {@link java.util.TreeMap}s: the keys are monotonic, so
     * appends are O(1) and lookups are O(log n) binary searches — same asymptotics as the trees —
     * but with no boxed keys, no per-entry node objects, and no garbage on trim, since the backing
     * arrays are allocated once and slots are reused. Pose data is stored as primitive {@code double}
     * columns so neither storing history nor reading it back allocates {@code Pose} objects.
     */
    private static final class History {
        private final long[] time;
        private final double[] fusedX, fusedY, fusedH;
        private final double[] odomX, odomY, odomH;
        private final Matrix[] cov;
        private final int capacity;
        private int head; // logical index 0 lives at array index `head`
        private int size;

        History(int maxEntries) {
            // +2 headroom absorbs a transient interior insert (a vision sample) before the count cap
            // is re-applied, so the arrays never have to grow.
            capacity = Math.max(maxEntries, 1) + 2;
            time = new long[capacity];
            fusedX = new double[capacity]; fusedY = new double[capacity]; fusedH = new double[capacity];
            odomX = new double[capacity]; odomY = new double[capacity]; odomH = new double[capacity];
            cov = new Matrix[capacity];
        }

        boolean isEmpty() { return size == 0; }
        int size() { return size; }
        long firstTime() { return time[head]; }
        long lastTime() { return time[arr(size - 1)]; }

        double odomXAt(int i) { return odomX[arr(i)]; }
        double odomYAt(int i) { return odomY[arr(i)]; }
        double odomHAt(int i) { return odomH[arr(i)]; }
        Matrix covAt(int i) { return cov[arr(i)]; }
        double lastFusedX() { return fusedX[arr(size - 1)]; }
        double lastFusedY() { return fusedY[arr(size - 1)]; }
        double lastFusedH() { return fusedH[arr(size - 1)]; }
        Matrix lastCov() { return cov[arr(size - 1)]; }

        void setFused(int i, double x, double y, double h) { int p = arr(i); fusedX[p] = x; fusedY[p] = y; fusedH[p] = h; }
        void setOdom(int i, double x, double y, double h) { int p = arr(i); odomX[p] = x; odomY[p] = y; odomH[p] = h; }
        void setCov(int i, Matrix c) { cov[arr(i)] = c; }

        /** Array index backing logical index {@code i} (0 == oldest). */
        private int arr(int i) { return (head + i) % capacity; }

        private long timeAt(int i) { return time[arr(i)]; }

        /** Appends a strictly-newer sample at the back; evicts the oldest if somehow at capacity. */
        void append(long t, double x, double y, double h, double ox, double oy, double oh, Matrix c) {
            if (size == capacity) evictOldest();
            store(arr(size), t, x, y, h, ox, oy, oh, c);
            size++;
        }

        /** Inserts or fully overwrites a sample, keeping the buffer time-sorted. */
        void put(long t, double x, double y, double h, double ox, double oy, double oh, Matrix c) {
            int fi = floorIndex(t);
            if (fi >= 0 && timeAt(fi) == t) {
                store(arr(fi), t, x, y, h, ox, oy, oh, c);
            } else {
                insertAt(fi + 1, t, x, y, h, ox, oy, oh, c);
            }
        }

        /**
         * Applies a vision correction at {@code t}: overwrites fused + covariance if a sample already
         * exists there (leaving its odometry pose), otherwise inserts a new full row.
         */
        void putCorrection(long t, double x, double y, double h, double ox, double oy, double oh, Matrix c) {
            int fi = floorIndex(t);
            if (fi >= 0 && timeAt(fi) == t) {
                int p = arr(fi);
                fusedX[p] = x; fusedY[p] = y; fusedH[p] = h; cov[p] = c;
            } else {
                insertAt(fi + 1, t, x, y, h, ox, oy, oh, c);
            }
        }

        /** Largest logical index whose time is ≤ {@code ts}, or -1 if none (binary search). */
        int floorIndex(long ts) {
            int lo = 0, hi = size - 1, res = -1;
            while (lo <= hi) {
                int mid = (lo + hi) >>> 1;
                if (timeAt(mid) <= ts) { res = mid; lo = mid + 1; }
                else hi = mid - 1;
            }
            return res;
        }

        /** Smallest logical index whose time is ≥ {@code ts}, or {@code size} if none. */
        private int ceilingIndex(long ts) {
            int lo = 0, hi = size - 1, res = size;
            while (lo <= hi) {
                int mid = (lo + hi) >>> 1;
                if (timeAt(mid) >= ts) { res = mid; hi = mid - 1; }
                else lo = mid + 1;
            }
            return res;
        }

        boolean interpolateFusedInto(long ts, double[] out) { return interpolate(fusedX, fusedY, fusedH, ts, out); }
        boolean interpolateOdomInto(long ts, double[] out) { return interpolate(odomX, odomY, odomH, ts, out); }

        /** Linear interpolation of a pose column at {@code ts} into {@code out}; false if out of range. */
        private boolean interpolate(double[] colX, double[] colY, double[] colH, long ts, double[] out) {
            int lo = floorIndex(ts);
            int hi = ceilingIndex(ts);
            if (lo < 0 || hi >= size) return false;

            int lower = arr(lo);
            if (lo == hi) {
                out[0] = colX[lower]; out[1] = colY[lower]; out[2] = colH[lower];
                return true;
            }
            int upper = arr(hi);
            double ratio = (double) (ts - time[lower]) / (time[upper] - time[lower]);

            out[0] = colX[lower] + ratio * (colX[upper] - colX[lower]);
            out[1] = colY[lower] + ratio * (colY[upper] - colY[lower]);
            double headingDiff = MathFunctions.getSmallestAngleDifference(colH[upper], colH[lower]);
            out[2] = MathFunctions.normalizeAngle(colH[lower] + ratio * headingDiff);
            return true;
        }

        /** Drops samples older than the most recent sample at or before {@code cutoff}. */
        void trimOlderThan(long cutoff) {
            int floor = floorIndex(cutoff);
            for (int k = 0; k < floor; k++) evictOldest();
        }

        void enforceCountCap(int max) {
            while (size > max) evictOldest();
        }

        private void evictOldest() {
            cov[head] = null; // release the only reference type for GC
            head = (head + 1) % capacity;
            size--;
        }

        /** Writes a full row's columns at array index {@code p} (does not touch size/head). */
        private void store(int p, long t, double x, double y, double h, double ox, double oy, double oh, Matrix c) {
            time[p] = t;
            fusedX[p] = x; fusedY[p] = y; fusedH[p] = h;
            odomX[p] = ox; odomY[p] = oy; odomH[p] = oh;
            cov[p] = c;
        }

        /**
         * Inserts a row at logical position {@code pos}, shifting the (small) suffix right by one.
         * Callers keep {@code size <= capacity - 1} before inserting (the +2 headroom), so there is
         * always room for the new row without disturbing the logical indices.
         */
        private void insertAt(int pos, long t, double x, double y, double h,
                              double ox, double oy, double oh, Matrix c) {
            for (int i = size; i > pos; i--) {
                int dst = arr(i), src = arr(i - 1);
                store(dst, time[src], fusedX[src], fusedY[src], fusedH[src], odomX[src], odomY[src], odomH[src], cov[src]);
            }
            store(arr(pos), t, x, y, h, ox, oy, oh, c);
            size++;
        }
    }
}
