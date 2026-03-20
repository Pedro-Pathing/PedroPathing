package com.pedropathing.control;

@FunctionalInterface
public interface PIDFCoefficientSupplier {
    PIDFCoefficients get(double error);

    class PIDFPiecewiseNode {
        public final double threshold;
        public final PIDFCoefficients coefficients;

        public PIDFPiecewiseNode(double threshold, PIDFCoefficients coefficients) {
            this.threshold = threshold;
            this.coefficients = coefficients;
        }

        public PIDFPiecewiseNode(PIDFCoefficients coefficients) {
            this.threshold = Double.MAX_VALUE;
            this.coefficients = coefficients;
        }
    }

    static PIDFCoefficientSupplier piecewise(PIDFPiecewiseNode... nodes) {
        return error -> {
            for (PIDFPiecewiseNode node : nodes) {
                if (Math.abs(error) <= node.threshold) {
                    return node.coefficients;
                }
            }
            return nodes[nodes.length - 1].coefficients;
        };
    }
}
