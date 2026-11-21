package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.AutoAim;
import org.firstinspires.ftc.teamcode.util.Timer;

public abstract class BaseTeleOp extends LinearOpMode {
    protected Hardware robotHardware;
    protected Intake intake;
    protected Shooter shooter;
    protected Transfer transfer;
    protected GoBildaPinpointDriver pinpoint;
    protected GamepadEx controller1;

    // Which alliance we're on. Auto hands it over, or X or B picks it during init if auto didn't run.
    protected boolean isRedAlliance = false;

    // Where the robot starts. A spot number (for the autos), or one of these two.
    private static final int FROM_AUTO = -1;    // where autonomous ended
    private static final int FROM_CORNER = -2;  // the relocalize corner, when auto didn't run
    private int startChoice = FROM_CORNER;

    // Where the robot is on the field, updated every loop by updateInputs()
    protected double poseX = 0, poseY = 0;   // inches
    protected double poseHeading = 0;        // radians, counterclockwise is positive
    protected double fieldVelX = 0, fieldVelY = 0; // inches per second, in field directions
    protected double headingVel = 0;         // radians per second, counterclockwise is positive

    // How much each new velocity reading is smoothed with the last one.
    // 0 = no smoothing, closer to 1 = smoother but slower to react.
    private static final double VELOCITY_SMOOTHING = 0.5;

    private double lastPoseX, lastPoseY, lastPoseHeading;
    private long lastPoseNanos;
    private boolean havePose = false;

    // Auto-aim settings. TODO tune on the robot.
    // How fast the robot spins at full turning power, in degrees per second.
    // Measure it by spinning in place at full turn and reading "Turn speed" on telemetry.
    private static final double MAX_TURN_SPEED_DEG = 290;
    // How hard to turn per radian of aim error. Raise it if aiming is slow, lower it if it overshoots.
    private static final double AIM_KP = 1.5;
    // Slows the turn down as it catches up, so it doesn't swing past. Raise it if it wobbles back and forth.
    private static final double AIM_KD = 0.1;
    // Fastest the auto-aim is allowed to turn (0.7 = 70% turning power)
    private static final double AIM_MAX_TURN = 0.7;
    // How close the aim has to be before feeding, in degrees
    private static final double AIM_TOLERANCE_DEG = 2;

    protected final AutoAim solver = new AutoAim();

    // The aim direction from the last loop, used to tell how fast it's moving
    private double lastAimHeading = 0;
    private long lastAimNanos = 0;
    private double aimRate = 0; // radians per second
    private boolean haveAim = false;

    // Shooter results from the last loop, for telemetry
    protected boolean aimed = false;
    protected double aimErrorDeg = 0;
    protected boolean inScoringSpot = false;

    // Match time (in ms) when the last loop ended
    private double lastLoopEndMs = 0;

    // Last power sent to each drive motor (NaN means nothing has been sent yet)
    private final double[] lastDrivePower = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    // Smallest power change worth sending to a motor (0.01 = 1% power)
    private static final double POWER_TOLERANCE = 0.01;

    public void initHardware() {
        robotHardware = new Hardware(hardwareMap);
        intake = robotHardware.robotIntake;
        shooter = robotHardware.robotShooter;
        transfer = robotHardware.robotTransfer;
        pinpoint = robotHardware.robotPinpoint;
        controller1 = new GamepadEx(gamepad1);
    }

    public void updateInputs() {
        controller1.readButtons();
        // Press BACK with the robot pushed into the relocalize corner to reset the position and
        // heading completely. This fixes both field centric driving and the auto-aim.
        if (controller1.wasJustPressed(GamepadKeys.Button.BACK)) {
            relocalizeInCorner();
        }
        pinpoint.update(); // full read, the shooter needs position, not just heading
        updatePose();
    }

    // Sets the starting spot directly, for autos that always start in the same place.
    // The number is the spot's position in FieldConstants.START_NAMES, starting from 0.
    protected void useStart(int spot) {
        startChoice = spot;
    }

    // ===== Controls =====
    // All the driver buttons are here, so changing a control only means changing it in one place.
    // The left stick drives, and is read in each TeleOp.

    // Turning from the triggers. Right trigger turns right, left trigger turns left,
    // and pressing harder turns faster.
    protected double triggerTurn() {
        return controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                - controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }

    // Hold the right bumper to aim, spin up, and shoot
    protected boolean shootHeld() {
        return controller1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
    }

    // Hold the left bumper for intake in, hold B for intake out
    protected void updateIntake() {
        if (controller1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            intake.in();
        } else if (controller1.getButton(GamepadKeys.Button.B)) {
            intake.out();
        } else {
            intake.stop();
        }
    }

    // Runs during init until START is pressed. Takes the position autonomous handed over.
    // If there isn't one (auto didn't run, or the app restarted), the robot has to start in the
    // relocalize corner instead, and X or B picks the alliance.
    protected void getHandoffDuringInit() {
        if (PoseStorage.hasPose) {
            startChoice = FROM_AUTO;
            isRedAlliance = PoseStorage.isRed;
        } else {
            startChoice = FROM_CORNER;
        }

        while (opModeInInit()) {
            if (startChoice == FROM_AUTO) {
                telemetry.addData("Start", "Where auto ended (x %.1f, y %.1f, %.0f deg)",
                        PoseStorage.x, PoseStorage.y, PoseStorage.headingDeg);
            } else {
                if (gamepad1.x) isRedAlliance = false;
                if (gamepad1.b) isRedAlliance = true;
                telemetry.addLine("No position from auto. Put the robot in the relocalize corner.");
                telemetry.addLine("X for blue, B for red");
            }
            String alliance = "BLUE";
            if (isRedAlliance) alliance = "RED";
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Status", "INITIALIZED");
            telemetry.update();
        }
    }

    // Tells the Pinpoint where the robot is starting on the field. Call it right after START.
    protected void setStartPose() {
        if (startChoice == FROM_AUTO) {
            isRedAlliance = PoseStorage.isRed;
            setPose(PoseStorage.x, PoseStorage.y, PoseStorage.headingDeg);
            PoseStorage.clear(); // used up, so a restarted TeleOp won't reuse it
        } else if (startChoice == FROM_CORNER) {
            relocalizeInCorner();
        } else {
            isRedAlliance = FieldConstants.START_IS_RED[startChoice];
            setPose(FieldConstants.START_X[startChoice], FieldConstants.START_Y[startChoice],
                    FieldConstants.START_HEADING[startChoice]);
        }
    }

    // Resets the position and heading to the relocalize corner, for our alliance.
    // The robot has to be pushed flat into the corner when this runs.
    protected void relocalizeInCorner() {
        if (isRedAlliance) {
            setPose(FieldConstants.RED_CORNER_X, FieldConstants.RED_CORNER_Y, FieldConstants.RED_CORNER_HEADING);
        } else {
            setPose(FieldConstants.BLUE_CORNER_X, FieldConstants.BLUE_CORNER_Y, FieldConstants.BLUE_CORNER_HEADING);
        }
    }

    // Tells the Pinpoint exactly where the robot is, in inches and degrees
    private void setPose(double x, double y, double headingDeg) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDeg));
        poseX = x;
        poseY = y;
        poseHeading = Math.toRadians(headingDeg);
        fieldVelX = 0;
        fieldVelY = 0;
        headingVel = 0;
        havePose = false; // the next reading starts fresh, so the jump to the new spot isn't counted as speed
    }

    // The field heading that points away from our drivers, in radians
    protected double driverForwardHeading() {
        if (isRedAlliance) return Math.toRadians(FieldConstants.RED_DRIVER_FORWARD_HEADING);
        return Math.toRadians(FieldConstants.BLUE_DRIVER_FORWARD_HEADING);
    }

    // Our CELL that opens toward the half of the field the robot is on. The driver only
    // shoots when that one is facing up, so the code doesn't need to know which one is.
    protected boolean aimingAtAudienceCell() {
        return poseY < 72;
    }

    protected double targetX() {
        if (isRedAlliance) {
            if (aimingAtAudienceCell()) return FieldConstants.RED_AUDIENCE_CELL_X;
            return FieldConstants.RED_BACK_CELL_X;
        }
        if (aimingAtAudienceCell()) return FieldConstants.BLUE_AUDIENCE_CELL_X;
        return FieldConstants.BLUE_BACK_CELL_X;
    }

    protected double targetY() {
        if (isRedAlliance) {
            if (aimingAtAudienceCell()) return FieldConstants.RED_AUDIENCE_CELL_Y;
            return FieldConstants.RED_BACK_CELL_Y;
        }
        if (aimingAtAudienceCell()) return FieldConstants.BLUE_AUDIENCE_CELL_Y;
        return FieldConstants.BLUE_BACK_CELL_Y;
    }

    // For telemetry
    protected String targetName() {
        if (aimingAtAudienceCell()) return "audience side CELL";
        return "back side CELL";
    }

    // Reads the position from the Pinpoint and works out how fast the robot is moving
    private void updatePose() {
        Pose2D pose = pinpoint.getPosition();
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);
        long now = System.nanoTime();

        if (havePose) {
            double dt = (now - lastPoseNanos) / 1e9;
            if (dt > 0) {
                double rawVelX = (x - lastPoseX) / dt;
                double rawVelY = (y - lastPoseY) / dt;
                double rawHeadingVel = wrapAngle(heading - lastPoseHeading) / dt;
                fieldVelX = VELOCITY_SMOOTHING * fieldVelX + (1 - VELOCITY_SMOOTHING) * rawVelX;
                fieldVelY = VELOCITY_SMOOTHING * fieldVelY + (1 - VELOCITY_SMOOTHING) * rawVelY;
                headingVel = VELOCITY_SMOOTHING * headingVel + (1 - VELOCITY_SMOOTHING) * rawHeadingVel;
            }
        }

        poseX = x;
        poseY = y;
        poseHeading = heading;
        lastPoseX = x;
        lastPoseY = y;
        lastPoseHeading = heading;
        lastPoseNanos = now;
        havePose = true;
    }

    // Runs the shooter for one loop. While shooting, it aims, spins up, and feeds once ready,
    // and returns the turn power the auto-aim wants (use it as rx). When not shooting, it
    // stops everything and returns 0.
    protected double updateShooter(boolean shooting) {
        aimed = false;
        aimErrorDeg = 0;
        inScoringSpot = AutoAim.canScoreFrom(poseX, poseY, targetX(), targetY());
        if (!shooting) {
            shooter.stop();
            transfer.stop();
            haveAim = false;
            aimRate = 0;
            return 0;
        }

        // Where to aim and how far to shoot, including the lead for moving
        solver.solve(poseX, poseY, fieldVelX, fieldVelY, targetX(), targetY());
        shooter.setForDistance(solver.distance);

        // How fast the aim direction is moving. When the robot drives past the target,
        // the aim keeps sweeping around, and the robot has to turn at that same speed to keep up.
        long now = System.nanoTime();
        if (haveAim) {
            double dt = (now - lastAimNanos) / 1e9;
            if (dt > 0) {
                double rawAimRate = wrapAngle(solver.aimHeading - lastAimHeading) / dt;
                aimRate = VELOCITY_SMOOTHING * aimRate + (1 - VELOCITY_SMOOTHING) * rawAimRate;
            }
        }
        lastAimHeading = solver.aimHeading;
        lastAimNanos = now;
        haveAim = true;

        // Turn toward the aim heading, in three parts. Keep up with the moving aim,
        // close the gap, and slow down as the gap closes.
        // These are counterclockwise positive, but positive rx turns right (clockwise),
        // so the command gets flipped at the end.
        double aimError = wrapAngle(solver.aimHeading - poseHeading);
        double keepUp = aimRate / Math.toRadians(MAX_TURN_SPEED_DEG);
        double closeGap = AIM_KP * aimError;
        double slowDown = AIM_KD * (aimRate - headingVel);
        double turnCommand = keepUp + closeGap + slowDown;

        aimErrorDeg = Math.toDegrees(aimError);
        aimed = Math.abs(aimErrorDeg) < AIM_TOLERANCE_DEG;

        // Only feed when the flywheel is at speed, the robot is pointed right, and it's on the
        // side of the HIVE where the shot can actually go into the opening
        if (aimed && shooter.isReady() && inScoringSpot) {
            transfer.feed();
        } else {
            transfer.stop();
        }

        return -Range.clip(turnCommand, -AIM_MAX_TURN, AIM_MAX_TURN);
    }

    // Shows position, and the shooter numbers while shooting
    protected void addShooterTelemetry(boolean shooting) {
        telemetry.addData("Aiming at", targetName());
        if (inScoringSpot) {
            telemetry.addData("Scoring spot", "YES, can score from here");
        } else {
            telemetry.addData("Scoring spot", "NO, move to the side the CELL opens toward");
        }
        telemetry.addData("Position (in)", "x %.1f  y %.1f", poseX, poseY);
        telemetry.addData("Turn speed (deg/s)", "%.0f", Math.toDegrees(headingVel));
        if (shooting) {
            telemetry.addData("Shot distance (in)", "%.1f", solver.distance);
            telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getRpm(), shooter.getTargetRpm());
            telemetry.addData("Aim error (deg)", "%.1f", aimErrorDeg);
            telemetry.addData("Ready", aimed && shooter.isReady() && inScoringSpot);
        }
    }

    // Keeps an angle between -180 and 180 degrees (in radians), so turns take the short way around
    protected static double wrapAngle(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    // Sends the four drive powers, skipping any motor whose power barely changed
    protected void setDrivePowers(double fl, double fr, double bl, double br) {
        setIfChanged(robotHardware.frontLeft, 0, fl);
        setIfChanged(robotHardware.frontRight, 1, fr);
        setIfChanged(robotHardware.backLeft, 2, bl);
        setIfChanged(robotHardware.backRight, 3, br);
    }

    // Only talks to the motor if the power changed by at least 1%, or if it's a stop
    private void setIfChanged(DcMotorEx motor, int index, double power) {
        double last = lastDrivePower[index];
        boolean firstTime = Double.isNaN(last);
        boolean stopping = power == 0 && last != 0; // a real 0 always goes through so the motor brakes
        boolean bigChange = Math.abs(power - last) >= POWER_TOLERANCE;
        if (firstTime || stopping || bigChange) {
            motor.setPower(power);
            lastDrivePower[index] = power;
        }
    }

    // Shows how long the last loop took, using the match timer
    protected void addLoopTimeTelemetry() {
        double now = Timer.milliseconds();
        telemetry.addData("Loop (ms)", "%.1f", now - lastLoopEndMs);
        lastLoopEndMs = now;
    }
}
