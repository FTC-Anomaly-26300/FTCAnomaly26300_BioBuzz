package org.firstinspires.ftc.teamcode.util;

// Carries the robot's position from autonomous into TeleOp.
//
// In a match, TeleOp starts wherever autonomous ended, not at a starting spot. So at the very
// end of autonomous, call PoseStorage.save(...) with the robot's position. TeleOp picks it up
// during init. Static variables stay in memory between OpModes until the Robot Controller app
// restarts, which is what makes this work.
public class PoseStorage {
    public static boolean hasPose = false;
    public static double x = 0;          // inches
    public static double y = 0;          // inches
    public static double headingDeg = 0; // degrees, counterclockwise is positive
    public static boolean isRed = false;

    // Call this at the end of autonomous. If the position comes from Pedro Pathing, its heading
    // is in radians, so pass Math.toDegrees(heading).
    public static void save(double xInches, double yInches, double headingDegrees, boolean redAlliance) {
        x = xInches;
        y = yInches;
        headingDeg = headingDegrees;
        isRed = redAlliance;
        hasPose = true;
    }

    // TeleOp calls this once it has used the saved position, so a restarted TeleOp
    // doesn't reuse an old one by mistake
    public static void clear() {
        hasPose = false;
    }
}
