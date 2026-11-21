package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auton.BaseAuto;

// Starts against the back wall, right in front of blue's back CELL. Shoots the 4 preloaded
// POLLEN, then drives forward off the wall for LEAVE. TeleOp then starts already lined up to shoot.
@Autonomous(name = "Auto Blue Front", group = "Auto", preselectTeleOp = "FieldCentric TeleOp")
public class AutoBlueFront extends BaseAuto {
    @Override
    protected int startSpot() {
        return 1; // "Blue Front" in FieldConstants.START_NAMES
    }

    @Override
    protected boolean isRedAuto() {
        return false;
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
