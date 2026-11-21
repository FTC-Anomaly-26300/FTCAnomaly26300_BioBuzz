package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.DriveInputFilter;
import org.firstinspires.ftc.teamcode.util.Timer;


@TeleOp(name = "RobotCentric TeleOp", group = "TeleOp")
public class RobotCentric extends BaseTeleOp{
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

            // Hold the right bumper to aim, spin up, and shoot
            boolean shooting = shootHeld();

            //<editor-fold desc="Drivetrain">
            double ly = smoother.shape(controller1.getLeftY());
            double lx = smoother.shape(controller1.getLeftX());
            double rx = smoother.shape(triggerTurn()); // turning is on the triggers

            // While shooting, the auto-aim does the turning, so the triggers are ignored
            if (shooting) {
                rx = 0;
            }

            smoother.update(ly, lx, rx);
            ly = smoother.out[0];
            lx = smoother.out[1];
            rx = smoother.out[2];

            // Aims, spins up, and feeds while shooting, and gives back the turn it wants
            double aimTurn = updateShooter(shooting);
            if (shooting) {
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

            updateIntake();

            //<editor-fold desc="Telemetry">
            addLoopTimeTelemetry();
            addShooterTelemetry(shooting);
            telemetry.update();
            //</editor-fold>
        }

    }

}
