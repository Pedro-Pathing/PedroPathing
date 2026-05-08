//package com.pedropathing.follower;
//
//import com.pedropathing.math.Pose;
//import com.pedropathing.math.Twist;
//import com.pedropathing.math.Velocity;
//import com.pedropathing.paths.Path;
//
//public final class FollowState {
//    private Pose pose = Pose.zero();
//    private Velocity velocity = Velocity.zero();
//    private Twist twist = Twist.zero();
//    private Path path;
//    private double tangentialSpeed = 0; //Signed
//    private double previousTime = System.nanoTime();
//    private double deltaTime;
//
//    public FollowState(Path path) {
//        this.path = path;
//    }
//
//    public void update(Pose pose, Velocity velocity, Twist twist) {
//        long nanoTime = System.nanoTime();
//        double delta = nanoTime - previousTime;
//        previousTime = nanoTime;
//
//        this.pose = pose;
//        this.velocity = velocity;
//        this.twist = twist;
//        tangentialSpeed = velocity.toLinear().dot(path.pathProgress.closestTangentVector);
//    }
//
//    public Pose getPose() {
//        return pose;
//    }
//
//    public Velocity getVelocity() {
//        return velocity;
//    }
//
//    public Twist getTwist() {
//        return twist;
//    }
//
//    public Path getPath() {
//        return path;
//    }
//
//    public double getTangentialSpeed() {
//        return tangentialSpeed;
//    }
//
//    public double getDeltaTime() {
//        return deltaTime;
//    }
//}
//
///*
// * Curve is BezierPoint, BezierLine, BezierCurve
// * Path(curve, curve, curve)
// * Path(path, path)
// */
