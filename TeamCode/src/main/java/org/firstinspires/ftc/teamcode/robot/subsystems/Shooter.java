package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.InterpTable;

public class Shooter {

    // Shooter motor is the goBILDA 5000 Series 12VDC motor (5000-0002-4008), with no gearbox.
    // Its encoder counts 28 ticks per revolution, and it tested at 5800 RPM with no load.
    private static final double TICKS_PER_REV = 28;
    private static final double FREE_SPEED_RPM = 5800;

    // Highest speed the code will ask for. Under load the motor can't reach its free speed,
    // so asking for more would mean it never counts as ready.
    private static final double MAX_RPM = 5300;

    // Speed control gains. The hub runs this loop itself, much faster than our code.
    // F is the main one, F = 32767 / (max ticks per second) = 32767 / (5800 x 28 / 60) = 12.1.
    // Then P = F / 10, I = P / 10, D = 0. These are the standard starting values.
    private static final double F = 32767 / (FREE_SPEED_RPM * TICKS_PER_REV / 60.0);
    private static final double P = F / 10;
    private static final double I = P / 10;
    private static final double D = 0;

    // The two set speeds the driver picks in TeleOp, square for a close shot and triangle for a
    // middle one. Circle (auto shoot) uses the distance table below instead.
    // The team's old code used 40% and 50% power. These are those powers times the motor's
    // 5800 RPM free speed, as a starting point. Unlike power, a set RPM stays the same as the
    // battery drains, so shots stay the same all match.
    // TODO check both with ShooterTuner from the close and middle spots, and use what scores.
    public static final double CLOSE_RPM = 2300;
    public static final double MIDDLE_RPM = 2900;

    // How close to the target speed counts as ready to shoot, in RPM
    private static final double READY_TOLERANCE_RPM = 75;

    // Smallest change in target speed worth sending to the motor, in RPM
    private static final double RPM_SEND_TOLERANCE = 10;

    // Distance to the target in inches, and the motor RPM that scores from there.
    // TODO replace these with real numbers measured with the ShooterTuner OpMode.
    private static final InterpTable RPM_BY_DISTANCE = new InterpTable(
            new double[]{24, 48, 72, 96, 120},
            new double[]{2400, 2800, 3200, 3600, 4000});

    private final DcMotorEx motor;
    private double targetRpm = 0;
    private double lastSentRpm = 0;

    public Shooter(DcMotorEx shooterMotor) {
        motor = shooterMotor;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
    }

    // Spins the flywheel to a set speed. 0 lets it coast to a stop.
    public void setRpm(double rpm) {
        rpm = Math.min(rpm, MAX_RPM);
        targetRpm = rpm;
        boolean stopping = rpm == 0 && lastSentRpm != 0;
        boolean bigChange = Math.abs(rpm - lastSentRpm) >= RPM_SEND_TOLERANCE;
        if (stopping || bigChange) {
            if (rpm == 0) {
                motor.setPower(0); // coasts down instead of braking hard
            } else {
                motor.setVelocity(rpm * TICKS_PER_REV / 60.0); // RPM to ticks per second
            }
            lastSentRpm = rpm;
        }
    }

    // Picks the speed from the table for a target this many inches away
    public void setForDistance(double inches) {
        setRpm(RPM_BY_DISTANCE.get(inches));
    }

    public void stop() {
        setRpm(0);
    }

    public double getRpm() {
        return motor.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    // True when the flywheel is spinning and close enough to the target speed to shoot
    public boolean isReady() {
        return targetRpm > 0 && Math.abs(getRpm() - targetRpm) <= READY_TOLERANCE_RPM;
    }
}
