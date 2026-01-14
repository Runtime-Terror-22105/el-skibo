package org.firstinspires.ftc.teamcode.math;

import java.util.ArrayList;
import java.util.List;

public class LinearInterpLUT {
    private List<Double> mX = new ArrayList<>();
    private List<Double> mY = new ArrayList<>();
    private boolean safeMode; // prevents an error from being thrown if a value outside of bounds is requested
    public boolean failOnNaN = true;

    public LinearInterpLUT(List<Double> input, List<Double> output) {
        this(input, output, false);
    }

    public LinearInterpLUT(List<Double> input, List<Double> output, boolean safeMode) {
        if (input == null || output == null || input.size() != output.size() || input.size() < 2) {
            throw new IllegalArgumentException("There must be at least two control "
                    + "points and the arrays must be of equal length.");
        }

        for (int i = 0; i < input.size(); i++) {
            mX.add(input.get(i));
            mY.add(output.get(i));
        }

        this.safeMode = safeMode;
    }

    public LinearInterpLUT() {
        this.safeMode = true;
    }

    /**
     * Adds a control point to the LUT
     * @param input the input value (x)
     * @param output the output value (y)
     * @return this class (for chaining calls)
     */
    public LinearInterpLUT add(double input, double output) {
        // Insertion sort to keep mX sorted
        int index = 0;
        while (index < mX.size() && mX.get(index) < input) {
            index++;
        }
        mX.add(index, input);
        mY.add(index, output);
        return this;
    }

    /**
     * No-op for compatibility with other LUT classes.
     * @return this class (for chaining calls)
     */
    public LinearInterpLUT createLUT() {
        // No-op
        return this;
    }

    /**
     * Interpolates the value of Y = f(X) for given X. Clamps X to the domain of the table.
     *
     * @param input The X value.
     * @return The interpolated Y = f(X) value.
     */
    public double get(double input) {
        if (Double.isNaN(input)) {
            if (failOnNaN) {
                throw new IllegalArgumentException("LinearInterpLUT input value is NaN.");
            } else {
                return input;
            }
        }

        double minX = mX.get(0);;
        double maxX = mX.get(mX.size() - 1);
        if (safeMode) {
            if (input <= minX) {
                return mY.get(0);
            }
            if (input >= maxX) {
                return mY.get(mY.size() - 1);
            }
        } else {
            if (input < minX || input > maxX) {
                throw new IllegalArgumentException("Input value " + input + " is out of bounds ["
                        + minX + ", " + maxX + "]");
            }
        }

        // Find the interval [x_i, x_{i+1}] that contains input
        int i = 0;
        while (i < mX.size() - 1 && input > mX.get(i + 1)) {
            i++;
        }

        // Linear interpolation
        double x0 = mX.get(i);
        double x1 = mX.get(i + 1);
        double y0 = mY.get(i);
        double y1 = mY.get(i + 1);
        if (x1 - x0 < 1e-8) {
            throw new IllegalArgumentException("Two control points have the same X value: " + x0);
        }
        double t = (input - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    public LinearInterpLUT setSafeMode(boolean safeMode) {
        this.safeMode = safeMode;
        return this;
    }

    public boolean getSafeMode() {
        return this.safeMode;
    }

    // For debugging.
    @Override
    public String toString() {
        StringBuilder str = new StringBuilder();
        final int n = mX.size();
        str.append("[");
        for (int i = 0; i < n; i++) {
            if (i != 0) {
                str.append(", ");
            }
            str.append("(").append(mX.get(i));
            str.append(", ").append(mY.get(i)).append(")");
        }
        str.append("]");
        return str.toString();
    }
}
