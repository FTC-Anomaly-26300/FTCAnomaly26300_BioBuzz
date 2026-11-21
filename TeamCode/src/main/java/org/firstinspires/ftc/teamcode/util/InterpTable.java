package org.firstinspires.ftc.teamcode.util;

// A table of measured points with a smooth curve through them.
// It finds the quadratic (value = a + b x key + c x key squared) that comes closest to every
// point, the same as a quadratic regression in Desmos. With 3 points the curve goes exactly
// through all of them. With more, it goes as close as it can to all of them, so one slightly
// off measurement doesn't throw off the rest.
// For example, shooter speeds measured at 24, 48, 72, 96 and 120 inches give a speed for 60 inches
// that follows the curve between them, instead of a straight line.
public class InterpTable {
    private final double minKey, maxKey;

    // The curve is worked out on the keys shifted and scaled to go from -1 to 1. Squaring big
    // numbers like 120 loses accuracy, and this avoids it. It doesn't change the curve's shape.
    private final double center, halfRange;

    // The curve is value = a + b x t + c x t squared, where t is the shifted and scaled key
    private final double a, b, c;

    // Keys must go from smallest to biggest, with one value for each key, and at least 3 points
    public InterpTable(double[] keys, double[] values) {
        if (keys.length != values.length || keys.length < 3) {
            throw new IllegalArgumentException("InterpTable needs at least 3 points, with one value for each key");
        }
        for (int i = 1; i < keys.length; i++) {
            if (keys[i] <= keys[i - 1]) {
                throw new IllegalArgumentException("InterpTable keys must go from smallest to biggest");
            }
        }
        minKey = keys[0];
        maxKey = keys[keys.length - 1];
        center = (minKey + maxKey) / 2;
        halfRange = (maxKey - minKey) / 2;

        // Add up the totals the best fit needs
        double n = keys.length;
        double sumT = 0, sumT2 = 0, sumT3 = 0, sumT4 = 0;
        double sumV = 0, sumTV = 0, sumT2V = 0;
        for (int i = 0; i < keys.length; i++) {
            double t = (keys[i] - center) / halfRange;
            double t2 = t * t;
            sumT += t;
            sumT2 += t2;
            sumT3 += t2 * t;
            sumT4 += t2 * t2;
            sumV += values[i];
            sumTV += t * values[i];
            sumT2V += t2 * values[i];
        }

        // The best fit is the a, b and c that solve these three equations together
        //   n     x a + sumT  x b + sumT2 x c = sumV
        //   sumT  x a + sumT2 x b + sumT3 x c = sumTV
        //   sumT2 x a + sumT3 x b + sumT4 x c = sumT2V
        // Solved with Cramer's rule, which uses determinants of 3 by 3 grids of these numbers.
        double det = det3(n, sumT, sumT2, sumT, sumT2, sumT3, sumT2, sumT3, sumT4);
        a = det3(sumV, sumT, sumT2, sumTV, sumT2, sumT3, sumT2V, sumT3, sumT4) / det;
        b = det3(n, sumV, sumT2, sumT, sumTV, sumT3, sumT2, sumT2V, sumT4) / det;
        c = det3(n, sumT, sumV, sumT, sumT2, sumTV, sumT2, sumT3, sumT2V) / det;
    }

    public double get(double key) {
        // Outside the table, use the closest end instead of guessing past what was measured.
        // A curve keeps bending past the last point, so it can go way off out there.
        double k = Math.max(minKey, Math.min(maxKey, key));
        double t = (k - center) / halfRange;
        return a + b * t + c * t * t;
    }

    // Determinant of the 3 by 3 grid
    //   p q r
    //   s u v
    //   w y z
    private static double det3(double p, double q, double r,
                               double s, double u, double v,
                               double w, double y, double z) {
        return p * (u * z - v * y) - q * (s * z - v * w) + r * (s * y - u * w);
    }
}
