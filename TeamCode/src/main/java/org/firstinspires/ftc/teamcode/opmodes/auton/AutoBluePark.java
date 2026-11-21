package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.FieldConstants;

// Starts against the blue wall beside the LOADING ZONE, out of the way of a partner robot.
// Drives 5 in off the wall for LEAVE (3 points), then strafes left into the LOADING ZONE
// for PARK (5 points). The zone is 11 in deep, so the back of the robot ends 5 in from the
// wall, well inside it, with room to spare if the robot stops a little short or long.
@Autonomous(name = "Auto Blue Park", group = "Auto", preselectTeleOp = "FieldCentric TeleOp")
public class AutoBluePark extends BaseAuto {
    @Override
    protected int startSpot() {
        return 0; // "Blue Park" in FieldConstants.START_NAMES
    }

    @Override
    protected boolean isRedAuto() {
        return false;
    }

    @Override
    protected double forwardDistance() {
        return 5;
    }

    @Override
    protected double strafeLeftDistance() {
        // From the start spot over to the middle of the LOADING ZONE
        return Math.abs(FieldConstants.BLUE_LOADING_ZONE_Y - FieldConstants.START_Y[startSpot()]);
    }
}
