package com.pedropathing.revhub.localizers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;

final class FakeDcMotorEx implements InvocationHandler {
    int position;
    DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    int stopAndResetCount;
    int getCurrentPositionCount;
    DcMotor.RunMode mode;
    private final DcMotorEx motor;

    FakeDcMotorEx() {
        motor = (DcMotorEx) Proxy.newProxyInstance(
                DcMotorEx.class.getClassLoader(),
                new Class<?>[] {DcMotorEx.class},
                this);
    }

    DcMotorEx motor() {
        return motor;
    }

    @Override
    public Object invoke(Object proxy, Method method, Object[] args) {
        String name = method.getName();
        if ("getCurrentPosition".equals(name)) {
            getCurrentPositionCount++;
            return position;
        }
        if ("getDirection".equals(name)) {
            return direction;
        }
        if ("setDirection".equals(name)) {
            direction = (DcMotorSimple.Direction) args[0];
            return null;
        }
        if ("setMode".equals(name)) {
            mode = (DcMotor.RunMode) args[0];
            if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                stopAndResetCount++;
                position = 0;
            }
            return null;
        }
        if ("equals".equals(name)) {
            return proxy == args[0];
        }
        if ("hashCode".equals(name)) {
            return System.identityHashCode(proxy);
        }
        if ("toString".equals(name)) {
            return "FakeDcMotorEx";
        }
        Class<?> type = method.getReturnType();
        if (type == Void.TYPE) {
            return null;
        }
        if (type == boolean.class) {
            return false;
        }
        if (type == byte.class) {
            return (byte) 0;
        }
        if (type == short.class) {
            return (short) 0;
        }
        if (type == int.class) {
            return 0;
        }
        if (type == long.class) {
            return 0L;
        }
        if (type == float.class) {
            return 0f;
        }
        if (type == double.class) {
            return 0d;
        }
        if (type == char.class) {
            return '\0';
        }
        return null;
    }
}
