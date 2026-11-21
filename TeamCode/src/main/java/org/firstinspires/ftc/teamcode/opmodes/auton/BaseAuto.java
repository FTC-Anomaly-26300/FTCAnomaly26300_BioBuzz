package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmodes.teleop.BaseTeleOp;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

// Shared code for the autos. Shoots the preloads if the auto asks for it, drives forward off
// the wall, then strafes left if the auto asks for it, and saves where it ended so TeleOp
// knows where the robot is. The Pinpoint measures
// the distance and keeps the robot pointed straight, so it doesn't depend on battery level,
// and it fixes the robot pulling to one side.
//
// This isn't an OpMode by itself. AutoRedFront, AutoRedPark, AutoBlueFront and AutoBluePark
// each pick a starting spot, so on match day the drive team only has to pick the right auto.
public abstract class BaseAuto extends BaseTeleOp {

    // Which starting spot this auto uses, its position in FieldConstants.START_NAMES
    protected abstract int startSpot();

    // Which alliance this auto is for. If the spot doesn't match, the auto refuses to run,
    // so a reordered list in FieldConstants can never send a red auto to a blue spot.
    protected abstract boolean isRedAuto();

    // How far to drive forward off the wall, in inches. LEAVE (3 points) only needs the robot
    // to stop touching the wall.
    protected abstract double forwardDistance();

    // How far to strafe left after driving forward, in inches. 0 means no strafe.
    protected double strafeLeftDistance() {
        return 0;
    }

    // Whether this auto shoots its 4 preloaded POLLEN before driving off the wall
    protected boolean shootsPreloads() {
        return false;
    }

    // How long the auto spends shooting, in seconds. It aims and spins up first, then feeds
    // whenever it's ready, the same as holding the shoot button in TeleOp. So this has to cover
    // the spin up plus all 4 shots. If POLLEN is still left in the robot after auto, raise it.
    private static final double SHOOT_SECONDS = 6;

    // Driving power (0.4 = 40%). Kept low so it doesn't slip or overshoot.
    private static final double DRIVE_POWER = 0.4;

    // Over the last few inches of each move it slows down, so it stops close to the right spot
    private static final double SLOW_DOWN_DISTANCE = 6;
    private static final double MIN_POWER = 0.2;

    // How hard to steer back when the robot turns off straight, per radian
    private static final double HEADING_KP = 1.0;
    private static final double MAX_CORRECTION = 0.3;

    // Gives up on a move after this many seconds, in case the robot gets stuck against something
    private static final double MOVE_TIMEOUT_SECONDS = 4;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        // An old saved position from an earlier run gets thrown away
        PoseStorage.clear();

        boolean spotMatches = FieldConstants.START_IS_RED[startSpot()] == isRedAuto();
        if (!spotMatches) {
            telemetry.addLine("STOP. This auto's starting spot is on the other alliance.");
            telemetry.addLine("Check the spot numbers in FieldConstants. The robot won't move.");
            telemetry.update();
            waitForStart();
            return;
        }

        useStart(startSpot());
        String alliance = "BLUE";
        if (isRedAuto()) alliance = "RED";
        telemetry.addData("Start", FieldConstants.START_NAMES[startSpot()]);
        telemetry.addData("Alliance", alliance);
        telemetry.addLine("Check this matches where the robot is before pressing START");
        telemetry.update();

        waitForStart();
        setStartPose();

        double holdHeading = poseHeading;
        if (shootsPreloads()) {
            shoot();
        }
        move(1, 0, forwardDistance(), holdHeading);  // forward
        move(0, -1, strafeLeftDistance(), holdHeading); // strafe left
        setDrivePowers(0, 0, 0, 0);

        // Keep saving the position until auto ends, so it includes any coasting after the stop
        while (opModeIsActive()) {
            updateInputs();
            PoseStorage.save(poseX, poseY, Math.toDegrees(poseHeading), isRedAlliance);
            telemetry.addData("Done", "x %.1f  y %.1f  heading %.0f", poseX, poseY, Math.toDegrees(poseHeading));
            telemetry.update();
        }
        PoseStorage.save(poseX, poseY, Math.toDegrees(poseHeading), isRedAlliance);
    }

    // Shoots the preloads from where the robot is. Uses the same code as the TeleOp shoot button,
    // so it aims at our CELL, spins up to the speed for that distance, and feeds once it's ready.
    private void shoot() {
        ElapsedTime shootTimer = new ElapsedTime();
        while (opModeIsActive() && shootTimer.seconds() < SHOOT_SECONDS) {
            updateInputs();
            PoseStorage.save(poseX, poseY, Math.toDegrees(poseHeading), isRedAlliance);

            double rx = updateShooter(true);
            setDrivePowers(rx, -rx, rx, -rx); // only turning, to hold the aim

            telemetry.addData("Shooting", "%.1f of %.0f s", shootTimer.seconds(), SHOOT_SECONDS);
            addShooterTelemetry(true);
            telemetry.update();
        }
        updateShooter(false); // stops the shooter and transfer
        setDrivePowers(0, 0, 0, 0);
    }

    // Drives in a straight line for a distance, while holding a heading.
    // forwardDir and strafeDir pick the direction. (1, 0) is forward, (0, -1) is left.
    private void move(double forwardDir, double strafeDir, double distance, double holdHeading) {
        if (distance <= 0) return;

        double moveStartX = poseX;
        double moveStartY = poseY;
        ElapsedTime moveTimer = new ElapsedTime();

        while (opModeIsActive() && moveTimer.seconds() < MOVE_TIMEOUT_SECONDS) {
            updateInputs();
            // Saved every loop, so TeleOp still gets the position if auto gets cut off mid-move
            PoseStorage.save(poseX, poseY, Math.toDegrees(poseHeading), isRedAlliance);

            double traveled = Math.hypot(poseX - moveStartX, poseY - moveStartY);
            double remaining = distance - traveled;
            if (remaining <= 0) break;

            double power = DRIVE_POWER;
            if (remaining < SLOW_DOWN_DISTANCE) {
                power = Math.max(MIN_POWER, DRIVE_POWER * remaining / SLOW_DOWN_DISTANCE);
            }

            // Steer back toward the held heading. The error is counterclockwise positive,
            // but positive rx turns right (clockwise), so it gets flipped.
            double headingError = wrapAngle(holdHeading - poseHeading);
            double rx = -Range.clip(HEADING_KP * headingError, -MAX_CORRECTION, MAX_CORRECTION);

            // Same mecanum math as TeleOp. Positive strafe is right, so left is negative.
            double ly = forwardDir * power;
            double lx = strafeDir * power;
            setDrivePowers(ly + lx + rx, ly - lx - rx, ly - lx + rx, ly + lx - rx);

            telemetry.addData("Moved (in)", "%.1f of %.1f", traveled, distance);
            telemetry.addData("Heading error (deg)", "%.1f", Math.toDegrees(headingError));
            telemetry.update();
        }

        setDrivePowers(0, 0, 0, 0);
    }
}
