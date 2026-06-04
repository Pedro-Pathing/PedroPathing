package com.pedropathing.localization;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.Random;

/**
 * Races the frozen TreeMap baseline ({@link FusionLocalizerTreeMap}) against the ring-buffer
 * production {@link FusionLocalizer} over an identical update + delayed-measurement workload, and
 * reports wall time, per-operation heap allocation, and GC activity.
 * <p>
 * Run with: {@code ./gradlew :core:benchmarkFusion}. This is a plain {@code main} (not a test), so it
 * never runs as part of {@code :core:test}.
 * <p>
 * The headline metric is <b>bytes allocated per update</b> (via the HotSpot per-thread allocation
 * counter): it is deterministic and isolates the container overhead, since both implementations run
 * the identical fusion math and allocate the same {@code Pose}/{@code Matrix} values — only the
 * history container differs (TreeMap node + boxed Long key per put vs. a reused primitive ring).
 */
public final class FusionLocalizerBenchmark {
    private static final int WARMUP_RUNS = 3;
    private static final int MEASURED_RUNS = 5;
    private static final int UPDATES = 100_000;
    private static final int BUFFER_SIZE = 250;       // ~1 s of history at 200 Hz
    private static final long STEP_NANOS = 5_000_000L; // 5 ms => 200 Hz

    private static final Pose INITIAL_COV = new Pose(1, 1, 1);
    private static final Pose PROCESS_VAR = new Pose(0.01, 0.01, 0.01);
    private static final Pose MEASUREMENT_VAR = new Pose(0.5, 0.5, 0.5);

    public static void main(String[] args) {
        // Sanity: keep this honest — both must compute the same trajectory on the workload.
        double ringChecksum = run(() -> new RingSubject(new OdomStub()), new Metrics());
        double treeChecksum = run(() -> new TreeSubject(new OdomStub()), new Metrics());
        if (Math.abs(ringChecksum - treeChecksum) > 1e-6) {
            throw new IllegalStateException("Implementations diverged: ring=" + ringChecksum + " tree=" + treeChecksum);
        }

        for (int i = 0; i < WARMUP_RUNS; i++) {
            run(() -> new RingSubject(new OdomStub()), new Metrics());
            run(() -> new TreeSubject(new OdomStub()), new Metrics());
        }

        Metrics ring = new Metrics();
        Metrics tree = new Metrics();
        for (int i = 0; i < MEASURED_RUNS; i++) {
            settle();
            run(() -> new TreeSubject(new OdomStub()), tree);
            settle();
            run(() -> new RingSubject(new OdomStub()), ring);
        }

        report(tree, ring);
    }

    /** Runs one workload pass, recording metrics into {@code out}. Returns a pose checksum. */
    private static double run(SubjectFactory factory, Metrics out) {
        Subject subject = factory.create();
        OdomStub odom = subject.odom();
        subject.setStartPose(new Pose(0, 0, 0));

        Random rnd = new Random(12345);
        long[] clocks = new long[UPDATES];
        long clock = 0;
        double ox = 0, oy = 0, oh = 0;
        double checksum = 0;

        long gcCount0 = gcCount(), gcTime0 = gcTime(), alloc0 = threadAllocatedBytes(), t0 = System.nanoTime();

        for (int i = 0; i < UPDATES; i++) {
            clock += STEP_NANOS;
            clocks[i] = clock;
            ox += (rnd.nextDouble() - 0.5) * 2;
            oy += (rnd.nextDouble() - 0.5) * 2;
            oh = MathFunctions.normalizeAngle(oh + (rnd.nextDouble() - 0.5) * 0.2);
            odom.pose = new Pose(ox, oy, oh);

            subject.update(clock);

            // One getPose() per step (as a real consumer caches it), reused below.
            Pose p = subject.getPose();
            checksum += p.getX() + p.getY() + p.getHeading(); // defeat dead-code elimination

            if (i > 20 && i % 10 == 0) {
                long ts = clocks[i - 5];
                Pose meas = new Pose(
                        p.getX() + (rnd.nextDouble() - 0.5),
                        p.getY() + (rnd.nextDouble() - 0.5),
                        p.getHeading() + (rnd.nextDouble() - 0.5) * 0.1);
                subject.addMeasurement(meas, ts, MEASUREMENT_VAR);
            }
        }

        long wall = System.nanoTime() - t0;
        long alloc = threadAllocatedBytes() - alloc0;
        out.record(wall, alloc, gcCount() - gcCount0, gcTime() - gcTime0);
        return checksum;
    }

    private static void report(Metrics tree, Metrics ring) {
        System.out.println();
        System.out.println("FusionLocalizer: TreeMap baseline vs. ring buffer");
        System.out.printf("  workload: %,d updates, %d-entry buffer, %d measured runs%n",
                UPDATES, BUFFER_SIZE, MEASURED_RUNS);
        System.out.println("  (per-op figures are per update)");
        System.out.println();
        System.out.printf("  %-22s %16s %16s %10s%n", "metric", "TreeMap", "ring buffer", "change");
        line("wall time (ns/op)", tree.wallPerOp(), ring.wallPerOp());
        line("alloc (bytes/op)", tree.allocPerOp(), ring.allocPerOp());
        line("GC collections", tree.avgGcCount(), ring.avgGcCount());
        line("GC time (ms)", tree.avgGcTime(), ring.avgGcTime());
        System.out.println();
        if (tree.allocPerOp() > 0 && ring.allocPerOp() >= 0) {
            System.out.printf("  -> ring buffer allocates %.1f%% less per update (%,d B/op saved)%n",
                    100.0 * (tree.allocPerOp() - ring.allocPerOp()) / tree.allocPerOp(),
                    tree.allocPerOp() - ring.allocPerOp());
        }
        if (threadAllocatedBytes() < 0) {
            System.out.println("  (allocation counter unavailable on this JVM; bytes/op shown as -1)");
        }
    }

    private static void line(String label, double tree, double ring) {
        String change = tree == 0 ? "n/a" : String.format("%+.1f%%", 100.0 * (ring - tree) / tree);
        System.out.printf("  %-22s %16.1f %16.1f %10s%n", label, tree, ring, change);
    }

    /** Best-effort quiescing between measured runs so GC deltas are attributed to the right pass. */
    private static void settle() {
        System.gc();
        try { Thread.sleep(50); } catch (InterruptedException ignored) { Thread.currentThread().interrupt(); }
    }

    private static long threadAllocatedBytes() {
        java.lang.management.ThreadMXBean bean = ManagementFactory.getThreadMXBean();
        if (bean instanceof com.sun.management.ThreadMXBean) {
            com.sun.management.ThreadMXBean sun = (com.sun.management.ThreadMXBean) bean;
            if (sun.isThreadAllocatedMemorySupported()) {
                sun.setThreadAllocatedMemoryEnabled(true);
                return sun.getThreadAllocatedBytes(Thread.currentThread().getId());
            }
        }
        return -1;
    }

    private static long gcCount() {
        long c = 0;
        for (GarbageCollectorMXBean gc : ManagementFactory.getGarbageCollectorMXBeans()) {
            long n = gc.getCollectionCount();
            if (n > 0) c += n;
        }
        return c;
    }

    private static long gcTime() {
        long t = 0;
        for (GarbageCollectorMXBean gc : ManagementFactory.getGarbageCollectorMXBeans()) {
            long n = gc.getCollectionTime();
            if (n > 0) t += n;
        }
        return t;
    }

    // ---- subjects ---------------------------------------------------------------------------

    @FunctionalInterface
    private interface SubjectFactory { Subject create(); }

    /** Uniform handle over either implementation, with a settable clock. */
    private interface Subject {
        OdomStub odom();
        void setStartPose(Pose p);
        void update(long clock);
        void addMeasurement(Pose measured, long timestamp, Pose variance);
        Pose getPose();
    }

    private static final class RingSubject extends FusionLocalizer implements Subject {
        private final OdomStub odom;
        private long clock;
        RingSubject(OdomStub odom) { super(odom, INITIAL_COV, PROCESS_VAR, MEASUREMENT_VAR, BUFFER_SIZE); this.odom = odom; }
        @Override protected long currentTimeNanos() { return clock; }
        @Override public OdomStub odom() { return odom; }
        @Override public void update(long clock) { this.clock = clock; super.update(); }
    }

    private static final class TreeSubject extends FusionLocalizerTreeMap implements Subject {
        private final OdomStub odom;
        private long clock;
        TreeSubject(OdomStub odom) { super(odom, INITIAL_COV, PROCESS_VAR, MEASUREMENT_VAR, BUFFER_SIZE); this.odom = odom; }
        @Override protected long currentTimeNanos() { return clock; }
        @Override public OdomStub odom() { return odom; }
        @Override public void update(long clock) { this.clock = clock; super.update(); }
    }

    /** Minimal dead-reckoning stub; the benchmark sets {@link #pose} directly each step. */
    private static final class OdomStub implements Localizer {
        Pose pose = new Pose();
        private final Pose velocity = new Pose();
        @Override public void update() { }
        @Override public Pose getPose() { return pose; }
        @Override public Pose getVelocity() { return velocity; }
        @Override public Vector getVelocityVector() { return velocity.getAsVector(); }
        @Override public void setStartPose(Pose p) { pose = p; }
        @Override public void setPose(Pose p) { pose = p; }
        @Override public double getTotalHeading() { return pose.getHeading(); }
        @Override public double getForwardMultiplier() { return 1; }
        @Override public double getLateralMultiplier() { return 1; }
        @Override public double getTurningMultiplier() { return 1; }
        @Override public void resetIMU() { }
        @Override public double getIMUHeading() { return 0; }
        @Override public boolean isNAN() { return false; }
        @Override public double getAngularVelocity() { return 0; }
    }

    // ---- metric accumulation ----------------------------------------------------------------

    private static final class Metrics {
        private long runs;
        private long wallTotal;
        private long allocTotal;
        private long gcCountTotal;
        private long gcTimeTotal;

        void record(long wall, long alloc, long gcCount, long gcTime) {
            runs++;
            wallTotal += wall;
            allocTotal += alloc;
            gcCountTotal += gcCount;
            gcTimeTotal += gcTime;
        }

        double wallPerOp() { return runs == 0 ? 0 : (double) wallTotal / runs / UPDATES; }
        long allocPerOp() { return runs == 0 ? 0 : allocTotal / runs / UPDATES; }
        double avgGcCount() { return runs == 0 ? 0 : (double) gcCountTotal / runs; }
        double avgGcTime() { return runs == 0 ? 0 : (double) gcTimeTotal / runs; }
    }

    private FusionLocalizerBenchmark() { }
}
