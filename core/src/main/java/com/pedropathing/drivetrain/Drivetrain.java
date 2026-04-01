package com.pedropathing.drivetrain;

public interface Drivetrain {
    void drive(Powers powers);

    class Powers {
        public final double forward;
        public final double strafe;
        public final double turn;

        public Powers(double forward, double strafe, double turn) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }
    }
}
