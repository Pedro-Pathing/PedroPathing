package com.pedropathing.math;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Stack;
import java.util.Vector;

/**
 * PolyDiffUtil is a class that provides the utility of differentiating a polynomial coefficient array.
 * @author William Phomphakdee
 * @version 1.0.0 11/026/2025
 */
public class PolyDiffUtil {
    /**
     * A class to store how one coefficient of an array of coefficients maps to another.
     */
    public static class ElementReference {
        private final int colIdx;
        private final double multiplier;

        public ElementReference(int colIdx, double multiplier) {
            this.colIdx = colIdx;
            this.multiplier = multiplier;
        }

        public int getColIdx() {
            return colIdx;
        }

        public double getMultiplier() {
            return multiplier;
        }

        @Override
        public String toString() {
            return "{from: " + this.colIdx + " , multiplier: " + this.multiplier + "}";
        }
    }

    /**
     * Ghetto sparse representation of a differentiation matrix of various powers of said matrix with differentiation amount as the key.
     */
    private static final HashMap<Integer, java.util.Vector<ElementReference>> differentiationMappings = new HashMap<>();

    /**
     * A method to check if the mapping for the requested diff level and poly coefficient count exists.
     * @param diffLevel differentiation level (1 for first derivative, 2 for second derivative... ect)
     * @param elementCount coefficient count (for example, [1, 2, 3] represents 1 + 2x + 3x^2)
     * @return boolean array of length 2 -- [true / false, -] represents if the diff level is cached; [true, true / false] represents if the number of coefficients can be retrieved
     */
    public static boolean[] hasMapping(int diffLevel, int elementCount){
        boolean[] output = {false, false};
        output[0] = differentiationMappings.containsKey(diffLevel);
        if (output[0]){
            output[1] = differentiationMappings.get(diffLevel).size() >= elementCount;
        }
        return output;
    }

    /**
     * Create a list of coefficients of an n-th derivative of a polynomial.
     * @param diffLevel first, second, third... derivative
     * @param elementCount number of coefficients (dimension of the polynomial coefficient vector space)
     * @return a <code>Vector</code> of <code>ElementReference</code>s
     */
    public static java.util.Vector<ElementReference> generateMappings(int diffLevel, int elementCount){
        return PolyDiffUtil.generateMappings(0, diffLevel, elementCount);
    }

    /**
     * Generate a vector of coefficients of an n-th derivative of a polynomial starting at
     * a passed in index of the coefficient vector up to the final coefficient index - diff level.
     * @param idxOffset start at where (1 for starting at linear term, 2 to start at quadratic term... etc.)
     * @param diffLevel first, second, third... derivative
     * @param elementCount number of coefficients (dimension of the polynomial coefficient vector space)
     * @return a <code>Vector</code> of <code>ElementReference</code>s
     * @throws IllegalArgumentException if the <code>elementCount</code> or <code>diffLevel</code> is under 0
     * @throws IndexOutOfBoundsException if the <code>idxOffset</code> is greater than or equal to the element count or less than 0
     */
    public static java.util.Vector<ElementReference> generateMappings(int idxOffset, int diffLevel, int elementCount){
        { // check params
            if (elementCount < 0){
                throw new IllegalArgumentException("Element count cannot be under 0. Current value: " + elementCount);
            }

            if (idxOffset < 0 || idxOffset >= elementCount){
                throw new IndexOutOfBoundsException("Index offset cannot be under 0 or past the element count. Current value: " + idxOffset);
            }

            if (diffLevel < 0){
                throw new IllegalArgumentException("Differentiation level cannot be under 0 (integration). Current value: " + diffLevel);
            }
        }


        Stack<ElementReference> elementReferences = new Stack<>();

        for (int i = diffLevel + idxOffset; i < elementCount; i++) {
            double element = 1;
            for (int j = 0; j < diffLevel; j++) {
                element *= i - j;
            }
            elementReferences.push(new ElementReference(i, element));

        }

        return elementReferences;
    }

    /**
     * A method to check and get the mappings based on the differentiation level and the length of
     * the original polynomial coefficient vector.
     * @param diffLevel first, second, third... derivative
     * @param elementCount number of coefficients (dimension of the polynomial coefficient vector space)
     * @return a <code>Vector</code> of <code>ElementReference</code>s whose length is <code>elementCount - diffLevel</code>
     */
    public static java.util.Vector<ElementReference> checkAndGetMappings(int diffLevel, int elementCount){
        boolean[] mappingsStatus = hasMapping(diffLevel, elementCount);

        java.util.Vector<ElementReference> output = new java.util.Vector<>();

        if (!mappingsStatus[0]) {
            differentiationMappings.put(diffLevel, generateMappings(diffLevel, elementCount));
            output.addAll(differentiationMappings.get(diffLevel));
            return output;
        }

        if (mappingsStatus[1]){
            java.util.Vector<ElementReference> actualMappings = differentiationMappings.get(diffLevel);
            for (int i = 0; i < elementCount - diffLevel; i++) {
                output.add(actualMappings.get(i));
            }
        } else {
            differentiationMappings.get(diffLevel).addAll(generateMappings(elementCount - differentiationMappings.get(diffLevel).size() + 1, diffLevel, elementCount));
            output.addAll(differentiationMappings.get(diffLevel));
        }

        return output;
    }

    /**
     * A method that accepts a polynomial's coefficients, perform the differentiation with
     * the output being a new array, and return that new array.
     * @param poly polynomial's coefficient array (listed as [1, 2, 3... n] for \sum_{n = 0}^{N} \left( a_{n} * x^{n} \right); 1 + 2x + 3x^2 + ... + a_n * x^n)
     * @param diffLevel first, second, third... derivative
     * @return a new array of coefficients as if the input poly has been differentiated (same length as input coefficient array)
     */
    public static double[] getPolyCoefficients(double[] poly, int diffLevel){
        if (diffLevel < 0){
            throw new IllegalArgumentException("Differentiation level cannot be under 0 (integration). Current value: " + diffLevel);
        }

        double[] output = new double[poly.length];

        // zero differentiation required; trivial copy to output array
        if (diffLevel == 0){
            System.arraycopy(poly, 0, output, 0, poly.length);
            return output;
        }

        Vector<ElementReference> mappings = PolyDiffUtil.checkAndGetMappings(diffLevel, poly.length);

        for (int i = 0; i < mappings.size(); i++) {
            ElementReference elementReference = mappings.get(i);
            output[i] = poly[elementReference.getColIdx()] * elementReference.getMultiplier();
        }
        return output;
    }

    /**
     * A method that computes the values of each coefficient for a t-value.
     * The sum of the output elements is the value of the polynomial at that t-value.
     * @param coefficients polynomial coefficients stored in an array (listed as [1, 2, 3... n] for \sum_{n = 0}^{N} \left( a_{n} * x^{n} \right); 1 + 2x + 3x^2 + ... + a_n * x^n)
     * @param t the independent variable (the x value in the examples)
     * @return an array of the values with the respective coefficients and t-value (if <code>coefficients</code> is [1, 3, 5, 7], the output is [1, 3t, 5t^2, 7t^3])
     */
    public static double[] polyValArr(double[] coefficients, double t){
        double[] output = new double[coefficients.length];
        output[0] = coefficients[0];

        double previousT = 1;

        for (int i = 1; i < output.length; i++) {
            previousT *= t;
            output[i] = previousT * coefficients[i];
        }

        return output;
    }

    /**
     * A variant of the <code>polyValArr</code> that takes in multiple t values instead of one
     * @param coefficients polynomial coefficients stored in an array (listed as [1, 2, 3... n] for \sum_{n = 0}^{N} \left( a_{n} * x^{n} \right); 1 + 2x + 3x^2 + ... + a_n * x^n)
     * @param t an array of t values
     * @return a array of an array of the values with the respective coefficients and t-value (if <code>coefficients</code> is [1, 3, 5, 7], the output is [1, 3t, 5t^2, 7t^3])
     */
    public static double[][] polyValArr(double[] coefficients, double[] t){
        double[][] output = new double[t.length][coefficients.length];
        double[] prevT = new double[t.length];
        Arrays.fill(prevT, 1);

        for (int col = 0; col < coefficients.length; col++) {
            for (int row = 0; row < t.length; row++) {
                output[row][col] = prevT[row] * coefficients[col];
                prevT[row] *= t[row];
            }
        }

        return output;
    }

    /**
     * A method that computes the output of a polynomial function when given the coefficients and x-value.
     * @param coefficients polynomial coefficients stored in an array (listed as [1, 2, 3... n] for \sum_{n = 0}^{N} \left( a_{n} * x^{n} \right); 1 + 2x + 3x^2 + ... + a_n * x^n)
     * @param t the independent variable (the x value in the examples)
     * @return the output of the polynomial function at a given t (or x)
     */
    public static double polyVal(double[] coefficients, double t){
        double output = 0;
        double prevT = 1;

        for (double coefficient : coefficients) {
            output += prevT * coefficient;
            prevT *= t;
        }

        return output;
    }
}
