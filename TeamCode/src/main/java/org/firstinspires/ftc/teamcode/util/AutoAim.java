package org.firstinspires.ftc.teamcode.util;

// Works out where to aim and how far the shot has to go, so a shot fired while the
// robot is moving still lands in the target.
//
// The idea is that a pollen leaves the shooter already moving with the robot. If the robot
// drives right while shooting, the pollen drifts right on its way to the target.
// So instead of aiming at the real target, we aim at a "lead point" shifted the
// opposite way by (robot velocity x flight time). The drift then carries the pollen
// from the lead point back onto the real target.
public class AutoAim {

    // Distance to the target in inches, and how long the pollen is in the air from there,
    // in seconds. TODO measure these, for example by filming shots in slow motion.
    private static final InterpTable FLIGHT_TIME_BY_DISTANCE = new InterpTable(
            new double[]{24, 48, 72, 96, 120},
            new double[]{0.30, 0.45, 0.60, 0.75, 0.90});

    // The flight time depends on the distance, and the distance depends on the flight time,
    // so we go back and forth a few times. 3 rounds is plenty for it to settle.
    private static final int ROUNDS = 3;

    // ===== Where a shot can go in =====
    // The up CELL's opening doesn't face straight up. It faces outward, toward the wall on its
    // side of the HIVE, tilted about 30 degrees up (manual Figure 10-3). So a CELL on the audience
    // side (Y below 72) opens toward the audience, and a CELL on the back side opens toward the back.
    // This check is kept as loose as possible on purpose. It only stops shots from behind the
    // CELL, which always hit its back, and doing that on purpose is likely a foul (G417).
    // Everything else is left to the driver.
    public static boolean canScoreFrom(double robotX, double robotY, double targetX, double targetY) {
        if (targetY < 72) {
            return robotY < targetY; // audience side CELL, robot has to be on the audience side of it
        }
        return robotY > targetY;     // back side CELL, robot has to be on the back side of it
    }

    // Results, updated every time solve() runs
    public double aimHeading = 0; // field heading to face, in radians
    public double distance = 0;   // distance to use for the shooter speed, in inches

    public void solve(double robotX, double robotY,
                      double velX, double velY,
                      double targetX, double targetY) {
        double leadX = targetX;
        double leadY = targetY;
        double d = Math.hypot(targetX - robotX, targetY - robotY);

        for (int i = 0; i < ROUNDS; i++) {
            double flightTime = FLIGHT_TIME_BY_DISTANCE.get(d);
            leadX = targetX - velX * flightTime;
            leadY = targetY - velY * flightTime;
            d = Math.hypot(leadX - robotX, leadY - robotY);
        }

        aimHeading = Math.atan2(leadY - robotY, leadX - robotX);
        distance = d;
    }
}
