package org.firstinspires.ftc.teamcode.math;

import java.lang.Math;

public class CubicEquation {
    private double a, b, c, d;

    public CubicEquation(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }


    public  double solveCubic() {
        // Constants and intermediate calculations
        double delta1 = (-Math.pow(b, 3) / (27 * Math.pow(a, 3))) + (b * c) / (6 * Math.pow(a, 2)) - (d / (2 * a));
        double delta2 = (-Math.pow(b, 3) / (27 * Math.pow(a, 3))) + (b * c) / (6 * Math.pow(a, 2)) - (d / (2 * a));
        double discriminant = Math.pow(delta1, 2) + Math.pow((c / (3 * a)) - (Math.pow(b, 2) / (9 * Math.pow(a, 2))), 3);
        // First cube root part

        double part1 = Math.cbrt(delta1 + Math.sqrt(discriminant));
        // Second cube root part
        double part2 = Math.cbrt(delta1 - Math.sqrt(discriminant));
        // Sum up the two parts (cube roots)
        double x = part1 + part2 - (b / (3 * a));
        return x;
    }

    public double findRoot(double guess) {
        double tolerance = 1e-7; // Tolerance for convergence
        int maxIterations = 1000;
        double x = guess;

        for (int i = 0; i < maxIterations; i++) {
            double fx = evaluate(x); // Value of f(x)
            double fPrimeX = evaluateDerivative(x); // Value of f'(x)

            if (Math.abs(fx) < tolerance) {
                return x; // Root found
            }

            if (Math.abs(fPrimeX) < tolerance) {
                return -100;
            }

            x = x - fx / fPrimeX; // Update using Newton-Raphson formula
        }
        return x;
    }




    public double evaluate(double x) {
        return a * Math.pow(x, 3) + b * Math.pow(x, 2) + c * x + d;
    }

    public double evaluateDerivative(double x) {
        return 3 * a * Math.pow(x, 2) + 2 * b * x + c;
    }

}
