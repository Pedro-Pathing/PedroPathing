package com.pedropathing.util;

public abstract class AutoActions extends Thread{
    public int pathNumber = 0;

    public AutoActions(int pathNumber) {
        this.pathNumber = pathNumber;
    }

    @Override
    public void run() {
        super.run();
        parallelAction();
    }
    public abstract void parallelAction();
    public abstract void linearAction();
}
