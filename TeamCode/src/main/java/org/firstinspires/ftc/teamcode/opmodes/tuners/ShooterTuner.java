package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.teleop.BaseTeleOp;
import org.firstinspires.ftc.teamcode.util.AutoAim;
import org.firstinspires.ftc.teamcode.util.Timer;

// Use this to fill in the RPM table in Shooter.
// 1. Drive to a spot, stop, and note the distance on the Driver Station.
// 2. Press Y to spin up, and change the speed with dpad up and down until shots score.
//    Shoot with the normal buttons, left trigger runs the indexer and right bumper the servos.
// 3. Write the distance and RPM down, then repeat at 4 or 5 distances from close to far.
@TeleOp(name = "Shooter Tuner", group = "Test")
public class ShooterTuner extends BaseTeleOp {

    // How much one dpad press changes the speed, in RPM
    private static final double RPM_STEP = 50;

    // Drive speed in the tuner, kept low so it's easy to line up (0.5 = half power)
    private static final double DRIVE_SCALE = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        getHandoffDuringInit(); // start in the relocalize corner, X or B picks the alliance

        waitForStart();
        Timer.start();
        setStartPose();

        double testRpm = 3000;
        boolean spinning = false;

        while (opModeIsActive()) {

            updateInputs();

            // Simple robot centric driving, no curve or smoothing
            double ly = controller1.getLeftY() * DRIVE_SCALE;
            double lx = controller1.getLeftX() * DRIVE_SCALE;
            double rx = controller1.getRightX() * DRIVE_SCALE;
            double frontLeftPower = ly + lx + rx;
            double frontRightPower = ly - lx - rx;
            double backLeftPower = ly - lx + rx;
            double backRightPower = ly + lx - rx;
            double maxPower = Math.max(1.0, Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
            setDrivePowers(frontLeftPower / maxPower, frontRightPower / maxPower,
                    backLeftPower / maxPower, backRightPower / maxPower);

            // Dpad up and down change the speed, Y turns the shooter on and off
            if (controller1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                testRpm += RPM_STEP;
            }
            if (controller1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                testRpm = Math.max(0, testRpm - RPM_STEP);
            }
            if (controller1.wasJustPressed(GamepadKeys.Button.Y)) {
                spinning = !spinning;
            }

            if (spinning) {
                shooter.setRpm(testRpm);
            } else {
                shooter.stop();
            }

            // Same as in a match. Left trigger runs the intake and indexer, left bumper reverses
            // them, and the right bumper runs the indexer servos to shoot.
            updateIntakeAndIndexer();

            double distance = Math.hypot(targetX() - poseX, targetY() - poseY);
            telemetry.addData("Aiming at", targetName());
            if (AutoAim.canScoreFrom(poseX, poseY, targetX(), targetY())) {
                telemetry.addData("Scoring spot", "YES");
            } else {
                telemetry.addData("Scoring spot", "NO, shots from here would hit the outside of the CELL");
            }
            telemetry.addData("Distance to target (in)", "%.1f", distance);
            telemetry.addData("Test RPM", "%.0f", testRpm);
            telemetry.addData("Actual RPM", "%.0f", shooter.getRpm());
            telemetry.addData("Shooter", spinning);
            telemetry.addLine("Y spin on/off, dpad up/down change RPM, left trigger and right bumper to shoot");
            telemetry.update();
        }
    }
}
