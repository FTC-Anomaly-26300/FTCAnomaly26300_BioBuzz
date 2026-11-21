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
    public DcMotorEx frontLeft, frontRight, backLeft, backRight, intakeMotor, outtakeMotor, transferMotor;
    public CRServo feederServo;
    public Intake robotIntake;
    public Shooter robotShooter;
    public Transfer robotTransfer;
    public GoBildaPinpointDriver robotPinpoint;

    // Init hardwareMaps
    public  Hardware(HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        feederServo = hardwareMap.get(CRServo.class, "feederServo");

        // Set motor zero power behavior to brake instead of move freely
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // flywheel coasts down
        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        outtakeMotor.setDirection(DcMotorEx.Direction.FORWARD);  // TODO positive speed has to shoot outward
        transferMotor.setDirection(DcMotorEx.Direction.FORWARD); // TODO positive power has to move pollen up

        robotIntake = new Intake(intakeMotor);
        robotShooter = new Shooter(outtakeMotor);
        robotTransfer = new Transfer(transferMotor, feederServo);

        robotPinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        robotPinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        robotPinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robotPinpoint.setOffsets(0, 0, DistanceUnit.MM); // TODO now needed, position tracking depends on it
        robotPinpoint.resetPosAndIMU();

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
