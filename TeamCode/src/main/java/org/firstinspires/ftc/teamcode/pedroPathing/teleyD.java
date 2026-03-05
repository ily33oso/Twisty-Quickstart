package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class teleyD extends LinearOpMode {
// figure out "slow mode" for chasis
    //435 rpm for lift
    //encoders for lift

    // ================= HARDWARE =================
    DcMotor fr, fl, bl, br, rslide, lslide, lintake, rintake;
    Servo rclaw, lclaw, rclamp, lclamp;
    IMU imu;

    // ================= DRIVE CONSTANTS =================
    final double DRIVE_POWER_MAX = 0.9;

    // ================= LIFT CONSTANTS =================
    final double TICKS_PER_MM = 48.0625; // figure out the max length of the slides
    final double MAX_MM = 285.04551365;
    final int MAX_TICKS = (int)(TICKS_PER_MM * MAX_MM);
    final int MIN_TICKS = 0;
    final double UP_POWER = .75;
    final double DOWN_POWER = -0.75;


    @Override
    public void runOpMode() {

        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");
        lslide = hardwareMap.get(DcMotor.class, "lslide");
        rslide = hardwareMap.get(DcMotor.class, "rslide");
        rintake = hardwareMap.get(DcMotor.class, "rintake");
        lintake = hardwareMap.get(DcMotor.class, "lintake");

        rclaw = hardwareMap.get(Servo.class, "rclaw");
        lclaw = hardwareMap.get(Servo.class, "lclaw");
        rclamp = hardwareMap.get(Servo.class, "rclamp");
        lclamp = hardwareMap.get(Servo.class, "lclamp");

        imu = hardwareMap.get(IMU.class, "imu");

        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);

        rintake.setDirection(DcMotor.Direction.FORWARD);
        lintake.setDirection(DcMotor.Direction.REVERSE);

        rslide.setDirection(DcMotorSimple.Direction.REVERSE);
        lslide.setDirection(DcMotorSimple.Direction.FORWARD);
        rslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rclaw.setDirection(Servo.Direction.REVERSE);
        lclaw.setDirection(Servo.Direction.FORWARD);
        rclamp.setDirection(Servo.Direction.REVERSE);
        lclamp.setDirection(Servo.Direction.FORWARD);

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        waitForStart();

        while (opModeIsActive()) {

            // ================= DRIVE =================
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotatedX = lx * Math.cos(heading) - ly * Math.sin(heading);
            double rotatedY = lx * Math.sin(heading) + ly * Math.cos(heading);

            double max = Math.max(Math.abs(rotatedX) + Math.abs(rotatedY) + Math.abs(rx), 1);
            double drivePower = DRIVE_POWER_MAX - (0.6 * gamepad1.right_trigger);

            br.setPower(((rotatedY - rotatedX - rx) / max) * drivePower);
            bl.setPower(((rotatedY + rotatedX + rx) / max) * drivePower);
            fr.setPower(((rotatedY + rotatedX - rx) / max) * drivePower);
            fl.setPower(((rotatedY - rotatedX + rx) / max) * drivePower);

            if (gamepad1.y) imu.resetYaw();

            // ================= INTAKE =================
            if (gamepad2.a) {
                lintake.setPower(1);
                rintake.setPower(1);
            } else {
                lintake.setPower(0);
                rintake.setPower(0);
            }


            // ================= CLAW =================
            if (gamepad2.right_bumper) {
                rclaw.setPosition(.23);
                lclaw.setPosition(.23);
            } else {
                rclaw.setPosition(0);
                lclaw.setPosition(0);
            }

            // ================= CLAMP =================
            if (gamepad1.left_bumper){
                rclamp.setPosition(.23);
                lclamp.setPosition(.23);
            } else {
                rclamp.setPosition(0);
                lclamp.setPosition(0);
            }

            // ================= LIFT =================
            int pos = lslide.getCurrentPosition();
            if (gamepad1.dpad_up && pos < MAX_TICKS) {
                lslide.setPower(UP_POWER);
                rslide.setPower(UP_POWER);
            }
            else if (gamepad1.dpad_down && pos > MIN_TICKS) {
                lslide.setPower(DOWN_POWER);
                rslide.setPower(DOWN_POWER);
            }
            else {
                lslide.setPower(0);
                rslide.setPower(0);
            }

            telemetry.update();
        }
    }
}
