package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class teletleley2 extends LinearOpMode {

    // ================= DRIVE =================
    DcMotor fr, fl, bl, br;

    // ================= MECHANISMS =================
    DcMotor rintake, lintake;

    Servo rclamp, lclamp;
    Servo rclaw, lclaw;

    IMU imu;

    // ================= CONSTANTS =================
    final double DRIVE_POWER_MAX = 0.9;

    @Override
    public void runOpMode() {

        // ================= HARDWARE =================
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");

        rintake = hardwareMap.get(DcMotor.class, "rintake");
        lintake = hardwareMap.get(DcMotor.class, "lintake");

        rclamp = hardwareMap.get(Servo.class, "rclamp");
        lclamp = hardwareMap.get(Servo.class, "lclamp");

        rclaw = hardwareMap.get(Servo.class, "rclaw");
        lclaw = hardwareMap.get(Servo.class, "lclaw");

        imu = hardwareMap.get(IMU.class, "imu");

        // ================= DIRECTIONS =================
        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);

        rintake.setDirection(DcMotor.Direction.FORWARD);
        lintake.setDirection(DcMotor.Direction.REVERSE);

        rclamp.setDirection(Servo.Direction.REVERSE);
        lclamp.setDirection(Servo.Direction.FORWARD);

        rclaw.setDirection(Servo.Direction.REVERSE);
        lclaw.setDirection(Servo.Direction.FORWARD);

        // ================= IMU =================
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        waitForStart();

        while (opModeIsActive()) {

            // ================= FIELD CENTRIC DRIVE =================
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
            if (gamepad1.a) {
                rintake.setPower(0.5);
                lintake.setPower(0.5);
            } else {
                rintake.setPower(0);
                lintake.setPower(0);
            }

            // ================= CLAMP =================
            if (gamepad1.left_bumper) {
                rclamp.setPosition(0.32);
                lclamp.setPosition(0.32);
            } else {
                rclamp.setPosition(0);
                lclamp.setPosition(0);
            }

            // ================= CLAW =================
            if (gamepad2.x) {
                rclaw.setPosition(0.1);
                lclaw.setPosition(0.1);
            } else {
                rclaw.setPosition(0);
                lclaw.setPosition(0);
            }

            telemetry.update();
        }
    }
}