package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;

public class Hardware {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight, intakeMotor, indexerMotor, outtakeMotor;
    public CRServo indexerServo1, indexerServo2;
    public Intake robotIntake;
    public Shooter robotShooter;
    public Transfer robotTransfer;
    public GoBildaPinpointDriver robotPinpoint;

    // Init hardwareMaps, with the Pinpoint
    public Hardware(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    // usePinpoint false skips the Pinpoint, so OpModes that don't need it (RobotCentric)
    // still run even if it's unplugged. robotPinpoint is null then.
    public Hardware(HardwareMap hardwareMap, boolean usePinpoint) {

        // These names have to match the robot configuration on the Control Hub exactly
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        indexerMotor = hardwareMap.get(DcMotorEx.class, "indexerMotor");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        indexerServo1 = hardwareMap.get(CRServo.class, "indexerServo1");
        indexerServo2 = hardwareMap.get(CRServo.class, "indexerServo2");

        // Set motor zero power behavior to brake instead of move freely
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        indexerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // flywheel coasts down

        // Directions from the code the team tested on the robot
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        indexerMotor.setDirection(DcMotorEx.Direction.FORWARD);  // positive power moves POLLEN up
        outtakeMotor.setDirection(DcMotorEx.Direction.REVERSE);  // positive speed shoots outward

        robotIntake = new Intake(intakeMotor);
        robotShooter = new Shooter(outtakeMotor);
        robotTransfer = new Transfer(indexerMotor, indexerServo1, indexerServo2);

        if (usePinpoint) {
            robotPinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            robotPinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
            robotPinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
            robotPinpoint.setOffsets(0, 0, DistanceUnit.MM); // TODO now needed, position tracking depends on it
            robotPinpoint.resetPosAndIMU();
        } else {
            robotPinpoint = null;
        }

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
