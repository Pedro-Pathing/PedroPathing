package com.pedropathing.localization;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.MatrixUtil;
import com.pedropathing.math.Vector;

import java.util.NavigableMap;
import java.util.TreeMap;

public class FusionLocalizer implements Localizer {
    private final Localizer deadReckoning;
    private Pose currentPosition;
    private Pose currentVelocity;
    private Matrix P; //State Covariance
    private final Matrix Q; //Process Noise Covariance
    private final Matrix R; //Measurement Noise Covariance
    private long lastUpdateTime = -1;
    private final NavigableMap<Long, Pose> poseHistory = new TreeMap<>();
    private final NavigableMap<Long, Pose> twistHistory = new TreeMap<>();
    private final NavigableMap<Long, Matrix> covarianceHistory = new TreeMap<>();
    private final int bufferSize;

    public FusionLocalizer(
            Localizer deadReckoning,
            double[] P,
            double[] processVariance,
            double[] measurementVariance,
            int bufferSize
    ) {
        this.deadReckoning = deadReckoning;
        this.currentPosition = new Pose();

        //Standard Deviations for Kalman Filter
        this.P = MatrixUtil.diag(P[0], P[1], P[2]);
        this.Q = MatrixUtil.diag(processVariance[0], processVariance[1], processVariance[2]);
        this.R = MatrixUtil.diag(measurementVariance[0], measurementVariance[1], measurementVariance[2]);
        this.bufferSize = bufferSize;
        twistHistory.put(0L, new Pose());
    }

    @Override
    public void update() {
        //Updates odometry
        deadReckoning.update();
        long now = System.nanoTime();
        double dt = lastUpdateTime < 0 ? 0 : (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;

        //Updates twist, note that the dead reckoning localizer returns world-frame twist
        Pose twist = deadReckoning.getVelocity();
        twistHistory.put(now, twist.copy());
        currentVelocity = twist.copy();

        //Perform twist integration to propagate the fused position estimate based on how the odometry thinks the robot has moved
        currentPosition = integrate(currentPosition, twist, dt);

        //Update Kalman Filter
        updateCovariance(dt);

        poseHistory.put(now, currentPosition.copy());
        covarianceHistory.put(now, P.copy());
        if (poseHistory.size() > bufferSize) poseHistory.pollFirstEntry();
        if (twistHistory.size() > bufferSize) twistHistory.pollFirstEntry();
        if (covarianceHistory.size() > bufferSize) covarianceHistory.pollFirstEntry();
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
    private void updateCovariance(double dt) {
        P = P.plus(Q.multiply(dt*dt));
    }

    /**
     * Adds a vision measurement
     * @param measuredPose the measured position by the camera, enter NaN to a specific axis if the camera couldn't measure that axis
     * @param timestamp the timestamp of the measurement
     */
    public void addMeasurement(Pose measuredPose, long timestamp) {
        if (!poseHistory.containsKey(timestamp)) return;

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
                {measH ? MathFunctions.normalizeAngle(
                        measuredPose.getHeading() - pastPose.getHeading()) : 0}
        });

        // Measurement mask M
        Matrix M = MatrixUtil.diag(
                measX ? 1 : 0,
                measY ? 1 : 0,
                measH ? 1 : 0
        );

        // Covariance at measurement time
        Matrix Pm = covarianceHistory.floorEntry(timestamp).getValue();

        // Innovation covariance S = P + R
        Matrix S = Pm.plus(R);

        Matrix K = Pm.multiply(invert(S));

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
        Matrix I = MatrixUtil.identity(3);
        Matrix IK = I.minus(K);
        Matrix updatedCovariance =
                IK.multiply(Pm).multiply(IK.transposed())
                        .plus(K.multiply(R).multiply(K.transposed()));

        covarianceHistory.put(timestamp, updatedCovariance);

        // Forward propagate pose + covariance
        long prevTime = timestamp;
        Pose prevPose = updatedPast;
        Matrix prevCov = updatedCovariance;

        for (NavigableMap.Entry<Long, Pose> entry :
                poseHistory.tailMap(timestamp, false).entrySet()) {

            long t = entry.getKey();
            Pose twist = interpolate(t, twistHistory);
            if (twist == null)
                twist = getVelocity();

            double dt = (t - prevTime) / 1e9;

            Pose nextPose = integrate(prevPose, twist, dt);
            poseHistory.put(t, nextPose);

            // Covariance propagation: P ← P + Q dt²
            prevCov = prevCov.plus(Q.multiply(dt * dt));
            covarianceHistory.put(t, prevCov);

            prevPose = nextPose;
            prevTime = t;
        }

        currentPosition = poseHistory.lastEntry().getValue().copy();
        P = covarianceHistory.lastEntry().getValue().copy();
    }

    //Inverts a matrix
    private Matrix invert(Matrix m) {
        if (m.getRows() != m.getColumns())
            throw new IllegalStateException("Matrix must be square");

        Matrix I = MatrixUtil.identity(m.getRows());
        Matrix[] r = Matrix.rref(m, I);

        if (!r[1].equals(I)) throw new IllegalArgumentException("matrix not invertible");
        return r[1];
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

    private Pose integrate(Pose previousPose, Pose twist, double dt) {
        //Standard forward-Euler first-order approximation for twist integration
        double dx = twist.getX() * dt;
        double dy = twist.getY() * dt;
        double dTheta = twist.getHeading() * dt;

        return new Pose(
                previousPose.getX() + dx,
                previousPose.getY() + dy,
                MathFunctions.normalizeAngle(previousPose.getHeading() + dTheta)
        );
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
        poseHistory.put(0L, setStart);
    }

    @Override
    public void setPose(Pose setPose) {
        currentPosition = setPose.copy();
        deadReckoning.setPose(setPose);

        if (poseHistory.lastEntry() != null)
            poseHistory.lastEntry().setValue(setPose.copy());
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
}