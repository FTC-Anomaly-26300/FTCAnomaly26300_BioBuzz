package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Moves pollen from the intake up to the shooter. The transfer motor carries them,
// and the gecko wheel servo gives the final push into the flywheel.
public class Transfer {

    // TODO tune these. Flip the sign if a part spins the wrong way.
    private static final double TRANSFER_POWER = 1.0;
    private static final double FEEDER_POWER = 1.0;

    private final DcMotorEx motor;
    private final CRServo feeder;

    private double lastMotorPower = 0;
    private double lastFeederPower = 0;

    public Transfer(DcMotorEx transferMotor, CRServo feederServo) {
        motor = transferMotor;
        feeder = feederServo;
    }

    // Sends pollen into the shooter
    public void feed() {
        set(TRANSFER_POWER, FEEDER_POWER);
    }

    public void stop() {
        set(0, 0);
    }

    // Only sends a command when the setting actually changes, like the intake
    private void set(double motorPower, double feederPower) {
        if (motorPower != lastMotorPower) {
            motor.setPower(motorPower);
            lastMotorPower = motorPower;
        }
        if (feederPower != lastFeederPower) {
            feeder.setPower(feederPower);
            lastFeederPower = feederPower;
        }
    }
}
