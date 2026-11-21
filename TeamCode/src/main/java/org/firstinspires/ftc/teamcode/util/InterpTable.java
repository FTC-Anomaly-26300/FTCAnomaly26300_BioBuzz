package org.firstinspires.ftc.teamcode.util;

// A table of measured points that fills in the values between them with straight lines.
// For example, "at 48 inches use 2800 RPM, at 72 inches use 3200 RPM" gives 3000 RPM at 60 inches.
public class InterpTable {
    private final double[] keys;
    private final double[] values;

    // Keys must go from smallest to biggest, with one value for each key
    public InterpTable(double[] keys, double[] values) {
        if (keys.length != values.length || keys.length < 2) {
            throw new IllegalArgumentException("InterpTable needs at least 2 points, with one value for each key");
        }
        for (int i = 1; i < keys.length; i++) {
            if (keys[i] <= keys[i - 1]) {
                throw new IllegalArgumentException("InterpTable keys must go from smallest to biggest");
            }
        }
        this.keys = keys.clone();
        this.values = values.clone();
    }

    public double get(double key) {
        // Outside the table, use the closest point instead of guessing past what was measured
        if (key <= keys[0]) return values[0];
        int last = keys.length - 1;
        if (key >= keys[last]) return values[last];

        // Find the two points the key sits between and draw a straight line between them
        int i = 1;
        while (key > keys[i]) i++;
        double t = (key - keys[i - 1]) / (keys[i] - keys[i - 1]); // 0 at the lower point, 1 at the upper
        return values[i - 1] + t * (values[i] - values[i - 1]);
    }
}
