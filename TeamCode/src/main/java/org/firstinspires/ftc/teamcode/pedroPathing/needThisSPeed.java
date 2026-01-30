package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Forward Intake Auto", group = "robot")
public class needThisSPeed extends LinearOpMode {

    // ===== DRIVETRAIN =====
    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intake;

    // ===== CONSTANTS =====
    static final double TICKS_PER_REV = 537.7;   // goBILDA 312RPM
    static final double WHEEL_DIAMETER_IN = 3.78;
    static final double GEAR_RATIO = 1.0;

    static final double COUNTS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);

    static final double DRIVE_POWER = 0.4;
    static final double INTAKE_RPM = -800;

    @Override
    public void runOpMode() {

        // ===== INIT =====
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        telemetry.addLine("READY: Back against goal line");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ===== START INTAKE =====
        intake.setVelocity((INTAKE_RPM / 60.0) * TICKS_PER_REV);

        // ===== DRIVE FORWARD 44.5 INCHES =====
        driveForwardInches(-44.5);

        // ===== HOLD INTAKE 1 MORE SECOND =====
        sleep(1000);

        // ===== STOP EVERYTHING =====
        stopDrive();
        intake.setVelocity(0);
    }

    private void driveForwardInches(double inches) {
        int target = (int) (inches * COUNTS_PER_INCH);

        fl.setTargetPosition(target);
        fr.setTargetPosition(target);
        bl.setTargetPosition(target);
        br.setTargetPosition(target);

        setRunToPosition();

        setDrivePower(DRIVE_POWER);

        while (opModeIsActive() && motorsBusy()) {
            telemetry.addData("Target Inches", inches);
            telemetry.update();
        }
    }

    private void resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunToPosition() {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setDrivePower(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    private boolean motorsBusy() {
        return fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy();
    }

    private void stopDrive() {
        setDrivePower(0);
    }
}

