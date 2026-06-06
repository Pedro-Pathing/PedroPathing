package com.pedropathing.localization;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

import java.util.NavigableMap;
import java.util.TreeMap;

public class FusionLocalizer implements Localizer {
    private static class KalmanState {
        Pose pose;
        Pose twist;
        Pose relativeTransform;
        Matrix covariance;

        public KalmanState(Pose pose, Pose twist, Pose relativeTransform, Matrix covariance) {
            this.pose = pose;
            this.twist = twist;
            this.relativeTransform = relativeTransform;
            this.covariance = covariance;
        }
    }

    private final Localizer deadReckoning;
    private Pose currentRawPose;
    private Pose currentPosition;
    private Pose currentVelocity;
    private Pose currentRelativeTransform;
    private Matrix P; //State Covariance
    private final Matrix Q; //Process Noise Covariance
    private final Matrix R; //Measurement Noise Covariance
    private long lastUpdateTime = -1;
    private final NavigableMap<Long, KalmanState> history = new TreeMap<>();
    private final int bufferSize;

    public FusionLocalizer(
            Localizer deadReckoning,
            Pose initialCovariance,
            Pose processVariance,
            Pose measurementVariance,
            int bufferSize
    ) {
        this.deadReckoning = deadReckoning;
        this.currentPosition = new Pose();
        currentRawPose = new Pose();

        //Standard Deviations for Kalman Filter
        this.P = Matrix.diag(initialCovariance.getX(), initialCovariance.getY(), initialCovariance.getHeading());
        this.Q = Matrix.diag(processVariance.getX(), processVariance.getY(), processVariance.getHeading());
        this.R = Matrix.diag(measurementVariance.getX(), measurementVariance.getY(), measurementVariance.getHeading());
        this.bufferSize = bufferSize;
        history.put(0L, new KalmanState(currentPosition, new Pose(), currentRawPose, P));
    }

    @Override
    public void update() {
        //Updates odometry
        deadReckoning.update();
        long now = System.nanoTime();
        double dt = lastUpdateTime < 0 ? 0 : (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;

        //Updates twist, note that the dead reckoning localizer returns world-frame twist
        currentVelocity = deadReckoning.getVelocity();

        //Update the pose estimate based on dead reckoning pose transformation
        Pose rawPose = deadReckoning.getPose();
        currentRelativeTransform = compose(invert(currentRawPose), rawPose);
        currentPosition = compose(currentPosition, currentRelativeTransform);
        currentRawPose = rawPose;

        //Update Kalman Filter
        updateCovariance(P, currentPosition, currentVelocity, dt);

        history.put(now, new KalmanState(currentPosition, currentVelocity, currentRelativeTransform, P));
        if (history.size() > bufferSize) history.pollFirstEntry();
    }

    /**
     * Consider the system xₖ₊₁ = xₖ + (f(xₖ, uₖ) + wₖ) * Δt.
     * <p>
     * wₖ is the noise in the system caused by sensor uncertainty, a zero-mean random vector with covariance Q.
     * <p>
     * The Kalman Filter update step is given by:
     * <pre>
     *     Pₖ₊₁ = F * Pₖ * Fᵀ + G * Q * Gᵀ
     * </pre>
     * Here F and G represent the State Transition Matrix and Control-to-State Matrix respectively.
     * <p>
     * The State Transition Matrix F is given by I + ∂f/∂x.
     * We computed our twist integration using a first-order forward-Euler approximation.
     * Therefore, f only depends on the twist, not on x, so ∂f/∂x = 0 and F = I.
     * <p>
     * The Control-to-State Matrix G is given by ∂xₖ₊₁ / ∂wₖ.
     * Here this is simply I * Δt.
     * <p>
     * The Kalman update is Pₖ₊₁ = F * Pₖ * Fᵀ + G * Q * Gᵀ.
     * With F = I and G = I * Δt, we get Pₖ₊₁ = Q * Δt².
     *
     * @param dt the time step Δt in seconds
     */
    private void updateCovariance(Matrix P, Pose pose, Pose twist, double dt) {
        double dist_x = Math.abs(twist.getX() * dt);
        double dist_y = Math.abs(twist.getY() * dt);
        double dist_theta = Math.abs(twist.getHeading() * dt);

        double q_x = Q.get(0, 0);
        double q_y = Q.get(1, 1);
        double q_theta = Q.get(2, 2);

        Matrix motionQ = Matrix.diag(
                dist_x * q_x,
                dist_y * q_y,
                dist_theta * q_theta
        );

        // Rotate into world frame
        Matrix Rtheta = Matrix.createRotation(pose.getHeading());
        Matrix worldQ = Rtheta.multiply(motionQ).multiply(Rtheta.transposed());

        P = P.plus(worldQ);
        clampCovariance(P);
    }

    private KalmanState getKalmanState() {
        return new KalmanState(currentPosition, currentVelocity, currentRelativeTransform, P);
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
        Matrix measurementR = measurementVariance == null ? R :
                Matrix.diag(measurementVariance.getX(), measurementVariance.getY(), measurementVariance.getHeading());
        // Reject if timestamp is outside our poseHistory time window
        if (history.isEmpty() || timestamp < history.firstKey() || timestamp > history.lastKey())
            return;

        KalmanState interpolatedData = interpolate(timestamp);
        if (interpolatedData == null) interpolatedData = getKalmanState();
        Pose pastPose = interpolatedData.pose;

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
        Matrix Pm = interpolatedData.covariance;

        // Innovation covariance S = P + R
        Matrix S = Pm.plus(measurementR);

        // Apply gain K = P * (P + R)^(-1)
        Matrix K = Pm.multiply(S.inverse());

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

        // Joseph-form covariance update
        Matrix I = Matrix.identity(3);
        Matrix IK = I.minus(K);
        Matrix cov = IK.multiply(Pm).multiply(IK.transposed()).plus(K.multiply(measurementR).multiply(K.transposed()));
        clampCovariance(cov);
        history.put(timestamp, new KalmanState(updatedPast, interpolatedData.twist, interpolatedData.relativeTransform, cov));

        // Forward propagate pose + covariance
        long prevTime = timestamp;
        Pose prevPose = updatedPast;

        for (NavigableMap.Entry<Long, KalmanState> entry : history.tailMap(timestamp, false).entrySet()) {
            long t = entry.getKey();
            Pose twist = entry.getValue().twist;
            if (twist == null) twist = getVelocity();

            double dt = (t - prevTime) / 1e9;

            Pose relativeTransform = entry.getValue().relativeTransform;
            updateCovariance(cov, prevPose, twist, dt);
            prevPose = compose(prevPose, relativeTransform);
            history.put(t, new KalmanState(prevPose, twist, entry.getValue().relativeTransform, P));
            prevTime = t;
        }

        currentPosition = history.lastEntry().getValue().pose;
        P = history.lastEntry().getValue().covariance;
    }

    private KalmanState interpolate(long timestamp) {
        Long lowerKey = history.floorKey(timestamp);
        Long upperKey = history.ceilingKey(timestamp);

        if (lowerKey == null || upperKey == null) return null;
        if (lowerKey.equals(upperKey)) return history.get(lowerKey);

        KalmanState lower = history.get(lowerKey);
        KalmanState upper = history.get(upperKey);
        Pose[] lowerKalmanState = new Pose[] {lower.pose, lower.twist, lower.relativeTransform};
        Pose[] upperKalmanState = new Pose[] {upper.pose, upper.twist, upper.relativeTransform};

        double ratio = (double) (timestamp - lowerKey) / (upperKey - lowerKey);

        Pose[] interpolData = new Pose[3];
        for (int i = 0; i < interpolData.length - 1; i++) {
            Pose lowerPose = lowerKalmanState[i];
            Pose upperPose = upperKalmanState[i];
            double x = lowerPose.getX() + ratio * (upperPose.getX() - lowerPose.getX());
            double y = lowerPose.getY() + ratio * (upperPose.getY() - lowerPose.getY());
            double headingDiff = MathFunctions.getSmallestAngleDifference(upperPose.getHeading(), lowerPose.getHeading());
            double heading = MathFunctions.normalizeAngle(lowerPose.getHeading() + ratio * headingDiff);
            interpolData[i] = new Pose(x, y, heading);
        }
        interpolData[2] = interpolateTransform(lowerKalmanState[2], upperKalmanState[2], ratio);

        return new KalmanState(interpolData[0], interpolData[1], interpolData[2], lower.covariance);
    }

    public static Pose invert(Pose pose) {
        double cos = Math.cos(pose.getHeading());
        double sin = Math.sin(pose.getHeading());

        double x = -pose.getX() * cos - pose.getY() * sin;
        double y =  pose.getX() * sin - pose.getY() * cos;
        double h = -pose.getHeading();

        return new Pose(x, y, h);
    }

    public static Pose compose(Pose a, Pose b) {
        double cos = Math.cos(a.getHeading());
        double sin = Math.sin(a.getHeading());

        double x = a.getX() + b.getX() * cos - b.getY() * sin;
        double y = a.getY() + b.getX() * sin + b.getY() * cos;
        double h = MathFunctions.normalizeAngle(a.getHeading() + b.getHeading());

        return new Pose(x, y, h);
    }

    public static Pose interpolateTransform(Pose a, Pose b, double ratio) {
        // 1. Linear interpolation in twist space
        double dx = a.getX() + ratio * (b.getX() - a.getX());
        double dy = a.getY() + ratio * (b.getY() - a.getY());
        double dtheta = a.getHeading() + ratio * (b.getHeading() - a.getHeading());

        // 2. Exponential map back to SE(2)
        double eps = 1e-4;
        double x, y;

        if (Math.abs(dtheta) < eps) {
            // Pure translation (small-angle approximation)
            x = dx;
            y = dy;
        } else {
            //twist integration
            double sinT = Math.sin(dtheta);
            double cosT = Math.cos(dtheta);
            double v = sinT / dtheta;
            double w = (1 - cosT) / dtheta;

            x = v * dx - w * dy;
            y = w * dx + v * dy;
        }

        return new Pose(x, y, dtheta);
    }

    private void clampCovariance(Matrix P) {
        double eps = 1e-6; // minimum allowed variance
        for (int i = 0; i < 3; i++) {
            double v = P.get(i, i);
            if (v < eps) {
                P.set(i, i, eps);
            }
        }
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
        history.put(0L, new KalmanState(setStart, new Pose(), setStart, P));
        currentPosition = setStart;
        currentRawPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        currentPosition = setPose;
        deadReckoning.setPose(setPose);
        currentRawPose = setPose;

        if (!history.isEmpty())
            history.lastEntry().getValue().pose = setPose;
        else
            setStartPose(setPose);
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