package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.DriveInputFilter;
import org.firstinspires.ftc.teamcode.util.Timer;

// Field centric driving with the driver's controls (listed in BaseTeleOp, under Controls).
// Pushing the left stick up always drives away from our drivers, whichever way the robot faces.
// RobotCentric is the main TeleOp. This one is kept as an option.
@TeleOp(name = "FieldCentric TeleOp", group = "TeleOp")
public class FieldCentric extends BaseTeleOp {
    private final DriveInputFilter smoother = new DriveInputFilter();

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        // Takes the position auto handed over (or the relocalize corner if auto didn't run)
        getHandoffDuringInit();

        // wait for user to press start button
        waitForStart();
        Timer.start();
        smoother.reset();
        setStartPose();

        // start OpMode loop
        while (opModeIsActive()) {

            updateInputs();

            // Hold circle for auto shoot
            boolean autoShoot = autoShootHeld();

            //<editor-fold desc="Drivetrain">
            // forward on the stick is away from our drivers, whichever way the robot faces
            double heading = poseHeading - driverForwardHeading();
            double ly = smoother.shape(controller1.getLeftY());
            double lx = smoother.shape(controller1.getLeftX());
            double rx = smoother.shape(controller1.getRightX());

            // During auto shoot the aim does the turning, so the right stick is ignored
            if (autoShoot) {
                rx = 0;
            }

            smoother.update(ly, lx, rx);
            ly = smoother.out[0];
            lx = smoother.out[1];
            rx = smoother.out[2];

            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double fieldForward = ly;
            double fieldRight = lx;
            ly = fieldForward * cos - fieldRight * sin;
            lx = fieldForward * sin + fieldRight * cos;

            // Runs the shooter, intake, indexer and servos, and gives back the turn auto shoot wants
            double aimTurn = updateShooterControls(autoShoot);
            if (autoShoot) {
                rx = aimTurn;
            }

            // Calculate motor powers
            double frontLeftPower = (ly + lx + rx);
            double frontRightPower = (ly - lx - rx);
            double backLeftPower = (ly - lx + rx);
            double backRightPower = (ly + lx - rx);

            // scale down if any motor is over 1.0
            double maxPower = Math.max(1.0, Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

            setDrivePowers(frontLeftPower / maxPower, frontRightPower / maxPower,
                    backLeftPower / maxPower, backRightPower / maxPower);
            //</editor-fold>

            //<editor-fold desc="Telemetry">
            addLoopTimeTelemetry();
            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            addShooterControlsTelemetry();
            telemetry.update();
            //</editor-fold>
        }
    }
}
