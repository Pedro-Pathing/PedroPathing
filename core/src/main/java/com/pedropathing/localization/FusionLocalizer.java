package com.pedropathing.localization;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Utils.Angle;

import java.util.NavigableMap;
import java.util.TreeMap;

public class FusionLocalizer implements Localizer {
    private static class KalmanState {
        Pose pose;
        Velocity velocity;
        Pose relativeTransform;
        Matrix covariance;

        public KalmanState(Pose pose, Velocity velocity, Pose relativeTransform, Matrix covariance) {
            this.pose = pose;
            this.velocity = velocity;
            this.relativeTransform = relativeTransform;
            this.covariance = covariance;
        }
    }

    public static double EPSILON = 1e-6; //floor for covariance matrices
    private final Localizer deadReckoning;
    private Pose currentRawPose;
    private MotionState motionState;
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
        motionState = MotionState.zero();
        currentRawPose = Pose.zero();

        //Standard Deviations for Kalman Filter
        this.P = Matrix.diag(
                Math.max(initialCovariance.x(), EPSILON),
                Math.max(initialCovariance.y(), EPSILON),
                Math.max(initialCovariance.heading(), EPSILON)
        );
        this.Q = Matrix.diag(
                Math.max(processVariance.x(), EPSILON),
                Math.max(processVariance.y(), EPSILON),
                Math.max(processVariance.heading(), EPSILON)
        );
        this.R = Matrix.diag(
                Math.max(measurementVariance.x(), EPSILON),
                Math.max(measurementVariance.y(), EPSILON),
                Math.max(measurementVariance.heading(), EPSILON)
        );
        this.bufferSize = bufferSize;
        history.put(0L, new KalmanState(Pose.zero(), Velocity.zero(), currentRawPose, P));
    }

    @Override
    public MotionState motionState() {
        return motionState;
    }

    @Override
    public void update() {
        //Updates odometry
        deadReckoning.update();
        long now = System.nanoTime();
        double dt = lastUpdateTime < 0 ? 0 : (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;

        //Updates twist, note that the dead reckoning localizer returns world-frame twist
        Velocity currentVelocity = deadReckoning.velocity();

        //Update the pose estimate based on dead reckoning pose transformation
        Pose rawPose = deadReckoning.pose();
        currentRelativeTransform = currentRawPose.invert().compose(rawPose);

        //Update Kalman states
        P = updateCovariance(P, motionState.pose(), currentVelocity, dt);
        Pose currentPosition = motionState.pose().compose(currentRelativeTransform);
        currentRawPose = rawPose;
        motionState = MotionState.ofVelocity(currentPosition, currentVelocity);

        history.put(now, new KalmanState(currentPosition, currentVelocity, currentRelativeTransform, P));
        if (history.size() > bufferSize) history.pollFirstEntry();
    }

    @Override
    public void reset() {
        deadReckoning.reset();
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
    private Matrix updateCovariance(Matrix P, Pose pose, Velocity velocity, double dt) {
        Twist twist = velocity.toTwist(pose.heading());
        Vector dist = twist.toVector().abs().times(dt);
        Vector Q_diag = Q.getDiagonal();
        Matrix bodyQ = Matrix.diag(dist.hadamardProduct(Q_diag));
        Matrix rotation = Matrix.rotationTransform(pose.heading());
        Matrix worldQ = rotation.times(bodyQ).times(rotation.transpose());
        P = P.plus(worldQ).clampDiagonals(EPSILON);
        return P;
    }

    private KalmanState getKalmanState() {
        return new KalmanState(motionState.pose(), motionState.velocity(), currentRelativeTransform, P);
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
                Matrix.diag(measurementVariance.x(), measurementVariance.y(), measurementVariance.heading()).clampDiagonals(EPSILON);

        // Reject if timestamp is outside our poseHistory time window
        if (history.isEmpty() || timestamp < history.firstKey() || timestamp > history.lastKey())
            return;

        KalmanState interpolatedData = interpolate(timestamp);
        if (interpolatedData == null) interpolatedData = getKalmanState();
        Pose pastPose = interpolatedData.pose;

        if (pastPose == null)
            pastPose = pose();

        // Measurement residual y = z - x
        boolean measX = !Double.isNaN(measuredPose.x());
        boolean measY = !Double.isNaN(measuredPose.y());
        boolean measH = !Double.isNaN(measuredPose.heading());

        Vector innovation = new Vector(
                measX ? measuredPose.x() - pastPose.x() : 0,
                measY ? measuredPose.y() - pastPose.y() : 0,
                measH ? Angle.normalizeSigned(measuredPose.heading() - pastPose.heading()) : 0
        );
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
        Matrix S_inv = S.invert();
        if (S_inv == null) return;
        Matrix K = Pm.times(S_inv);

        // Apply mask
        K = M.times(K);
        innovation = M.times(innovation);

        // State update
        Vector Ky = K.times(innovation);
        Pose updatedPast = new Pose(
                pastPose.x() + Ky.get(0),
                pastPose.y() + Ky.get(1),
                Angle.normalize(pastPose.heading() + Ky.get(2))
        );

        // Joseph-form covariance update
        Matrix I = Matrix.identity(3);
        Matrix IK = I.minus(K);
        Matrix cov = IK.times(Pm)
                .times(IK.transpose())
                .plus(K.times(measurementR).times(K.transpose()))
                .clampDiagonals(EPSILON);
        history.put(timestamp, new KalmanState(updatedPast, interpolatedData.velocity, interpolatedData.relativeTransform, cov));

        // Forward propagate pose + covariance
        long prevTime = timestamp;
        Pose prevPose = updatedPast;

        for (NavigableMap.Entry<Long, KalmanState> entry : history.tailMap(timestamp, false).entrySet()) {
            long t = entry.getKey();
            Velocity velocity = entry.getValue().velocity;
            if (velocity == null) velocity = velocity();

            double dt = (t - prevTime) / 1e9;

            Pose relativeTransform = entry.getValue().relativeTransform;
            cov = updateCovariance(cov, prevPose, velocity, dt);
            prevPose = prevPose.compose(relativeTransform);
            history.put(t, new KalmanState(prevPose, velocity, entry.getValue().relativeTransform, cov));
            prevTime = t;
        }

        motionState = MotionState.ofVelocity(history.lastEntry().getValue().pose, motionState.velocity());
        P = history.lastEntry().getValue().covariance;
    }

    private KalmanState interpolate(long timestamp) {
        Long lowerKey = history.floorKey(timestamp);
        Long upperKey = history.ceilingKey(timestamp);

        if (lowerKey == null || upperKey == null) return null;
        if (lowerKey.equals(upperKey)) return history.get(lowerKey);

        KalmanState lower = history.get(lowerKey);
        KalmanState upper = history.get(upperKey);

        double ratio = (double) (timestamp - lowerKey) / (upperKey - lowerKey);

        Velocity lowerVel = lower.velocity;
        Velocity upperVel = upper.velocity;
        Velocity interpolVel = lowerVel.plus(upperVel.minus(lowerVel).scale(ratio));
        Pose interpolPose = Pose.interpolate(lower.pose, upper.pose, ratio);
        Pose interpolTransform = interpolateTransform(lower.relativeTransform, upper.relativeTransform, ratio);
        return new KalmanState(interpolPose, interpolVel, interpolTransform, lower.covariance);
    }

    public static Pose interpolateTransform(Pose a, Pose b, double ratio) {
        //Linear interpolation in twist space
        Twist delta = Twist.riemannianLog(a, b).times(ratio);
        return a.compose(Pose.zero().exp(delta));
    }

    public void setStartPose(Pose setStart) {
        deadReckoning.setPose(setStart);
        history.put(0L, new KalmanState(setStart, Velocity.zero(), setStart, P));
        motionState = MotionState.ofVelocity(setStart, motionState.velocity());
        currentRawPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        motionState = MotionState.ofVelocity(setPose, motionState.velocity());
        deadReckoning.setPose(setPose);
        currentRawPose = setPose;

        if (!history.isEmpty())
            history.lastEntry().getValue().pose = setPose;
        else
            setStartPose(setPose);
    }
}