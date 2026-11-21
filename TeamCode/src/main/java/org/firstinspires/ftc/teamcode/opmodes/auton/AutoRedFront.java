package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Starts against the audience wall, right in front of red's audience CELL. Shoots the 4 preloaded
// POLLEN, then drives forward off the wall for LEAVE. TeleOp then starts already lined up to shoot.
@Autonomous(name = "Auto Red Front", group = "Auto", preselectTeleOp = "FieldCentric TeleOp")
public class AutoRedFront extends BaseAuto{
    @Override
    protected int startSpot() {
        return 3; // "Red Front" in FieldConstants.START_NAMES
    }

    @Override
    protected boolean isRedAuto() {
        return true;
    }

    @Override
    protected double forwardDistance() {
        return 12;
    }

    @Override
    protected boolean shootsPreloads() {
        return true;
    }
}
