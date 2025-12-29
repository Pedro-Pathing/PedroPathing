package com.pedropathing.localization;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
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
    private Matrix P;
    private final Matrix Q;
    private final Matrix R;
    private long lastUpdateTime = -1;

    private final NavigableMap<Long, Pose> poseHistory = new TreeMap<>();
    private final NavigableMap<Long, Pose> twistHistory = new TreeMap<>();
    private final NavigableMap<Long, Matrix> covarianceHistory = new TreeMap<>();
    private final int bufferSize;
    private final boolean useNonlinearDynamics;

    public FusionLocalizer(
            Localizer deadReckoning,
            double[] P,
            double[] processVariance,
            double[] measurementVariance,
            int bufferSize,
            boolean useNonlinearDynamics
    ) {
        this.deadReckoning = deadReckoning;
        this.useNonlinearDynamics = useNonlinearDynamics;
        this.currentPosition = new Pose();
        this.P = MatrixUtil.diag(P[0], P[1], P[2]);
        this.Q = MatrixUtil.diag(processVariance[0], processVariance[1], processVariance[2]);
        this.R = MatrixUtil.diag(measurementVariance[0], measurementVariance[1], measurementVariance[2]);
        this.bufferSize = bufferSize;
        twistHistory.put(0L, new Pose());
    }

    @Override
    public void update() {
        deadReckoning.update();
        long now = System.nanoTime();
        double dt = lastUpdateTime < 0 ? 0 : (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;

        Pose twist = deadReckoning.getVelocity();
        twistHistory.put(now, twist.copy());
        currentVelocity = twist.copy();

        double dx = twist.getX() * dt;
        double dy = twist.getY() * dt;
        double dTheta = twist.getHeading() * dt;

        currentPosition = new Pose(
                currentPosition.getX() + dx,
                currentPosition.getY() + dy,
                MathFunctions.normalizeAngle(currentPosition.getHeading() + dTheta)
        );

        updateCovariance(dt);

        poseHistory.put(now, currentPosition.copy());
        covarianceHistory.put(now, P.copy());
        if (poseHistory.size() > bufferSize) poseHistory.pollFirstEntry();
        if (twistHistory.size() > bufferSize) twistHistory.pollFirstEntry();
        if (covarianceHistory.size() > bufferSize) covarianceHistory.pollFirstEntry();
    }

    private void updateCovariance(double dt) {
        if (!useNonlinearDynamics) {
            P = P.plus(Q.multiply(dt));
        } else {
            Matrix F = MatrixUtil.identity(3);
            Matrix G = MatrixUtil.identity(3).multiply(dt);
            Matrix Q_d = G.multiply(Q).multiply(G.transposed());
            P = F.multiply(P).multiply(F.transposed()).plus(Q_d);
        }
    }

    public void addMeasurement(Pose measuredPose, long timestamp) {
        if (!poseHistory.containsKey(timestamp)) return;

        Pose pastPose = interpolate(timestamp, poseHistory);
        if (pastPose == null)
            pastPose = getPose();
        Matrix y = new Matrix(new double[][]{
                {!Double.isNaN(measuredPose.getX()) ? measuredPose.getX() - pastPose.getX() : 0},
                {!Double.isNaN(measuredPose.getY()) ? measuredPose.getY() - pastPose.getY() : 0},
                {!Double.isNaN(measuredPose.getHeading()) ?
                        MathFunctions.normalizeAngle(measuredPose.getHeading() - pastPose.getHeading()) : 0}
        });

        Matrix pastCovariance = covarianceHistory.floorEntry(timestamp).getValue();
        Matrix S = pastCovariance.plus(R);
        Matrix K = pastCovariance.multiply(invert(S));

        Matrix K_y = K.multiply(y);
        Pose updatedPast = new Pose(
                pastPose.getX() + K_y.get(0,0),
                pastPose.getY() + K_y.get(1,0),
                MathFunctions.normalizeAngle(pastPose.getHeading() + K_y.get(2,0))
        );
        poseHistory.put(timestamp, updatedPast);

        long previousTime = timestamp;
        Pose previousPose = updatedPast;
        double dt = 0;
        for (NavigableMap.Entry<Long, Pose> entry : poseHistory.tailMap(timestamp, false).entrySet()) {
            long t = entry.getKey();
            Pose twist = interpolate(t, twistHistory);
            if (twist == null)
                twist = getVelocity();
            dt = (t - previousTime) / 1e9;
            Pose nextPose = integrate(previousPose, twist, dt);
            poseHistory.put(t, nextPose);
            previousPose = nextPose;
            previousTime = t;
        }

        currentPosition = poseHistory.lastEntry().getValue();
        updateCovariance(dt);
        covarianceHistory.put(previousTime, P.copy());
    }

    private Matrix invert(Matrix m) {
        if (m.getRows() != m.getColumns())
            throw new IllegalStateException("Matrix must be square");

        Matrix I = MatrixUtil.identity(m.getRows());
        Matrix[] r = Matrix.rref(m, I);

        if (!r[1].equals(I)) throw new IllegalArgumentException("matrix not invertible");
        return r[1];
    }

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

    public boolean isUseNonlinearDynamics() { return useNonlinearDynamics; }
}
