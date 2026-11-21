package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Moves POLLEN from the intake up to the shooter. The indexer motor carries it,
// and the two indexer servos give the final push into the flywheel.
public class Transfer {

    // Powers from the code the team tested on the robot
    private static final double INDEXER_POWER = 1.0;
    private static final double SERVO_POWER = 0.5;

    private final DcMotorEx motor;
    private final CRServo servo1, servo2;

    private double lastMotorPower = 0;
    private double lastServoPower = 0;

    public Transfer(DcMotorEx indexerMotor, CRServo indexerServo1, CRServo indexerServo2) {
        motor = indexerMotor;
        servo1 = indexerServo1;
        servo2 = indexerServo2;
    }

    // Runs everything toward the shooter, used by the auto-aim when it's ready to shoot
    public void feed() {
        set(INDEXER_POWER, SERVO_POWER);
    }

    public void stop() {
        set(0, 0);
    }

    // Runs the indexer motor and the servos separately, for the driver's controls.
    // For each one, 1 runs it toward the shooter, -1 runs it backward, and 0 stops it.
    public void run(int indexerDirection, int servoDirection) {
        set(indexerDirection * INDEXER_POWER, servoDirection * SERVO_POWER);
    }

    // Only sends a command when the setting actually changes, like the intake
    private void set(double motorPower, double servoPower) {
        if (motorPower != lastMotorPower) {
            motor.setPower(motorPower);
            lastMotorPower = motorPower;
        }
        if (servoPower != lastServoPower) {
            servo1.setPower(servoPower);
            servo2.setPower(servoPower);
            lastServoPower = servoPower;
        }
    }
}
