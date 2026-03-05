package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class clampandbase extends OpMode {

    DcMotor fr, fl, br, bl;
    Servo rclamp, lclamp;
    IMU imu;

    double DRIVE_POWER_MAX = 0.9;

    @Override
    public void init() {
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");

        rclamp = hardwareMap.get(Servo.class, "rclamp");
        lclamp = hardwareMap.get(Servo.class, "lclamp");

        imu = hardwareMap.get(IMU.class, "imu");

        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);

        rclamp.setDirection(Servo.Direction.REVERSE);
        lclamp.setDirection(Servo.Direction.FORWARD);

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));
    }

    @Override
    public void loop() {

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

        // reset field heading
        if (gamepad1.y) {
            imu.resetYaw();
        }

        // ================= CLAMP =================
        if (gamepad1.left_bumper) {
            rclamp.setPosition(.32);
            lclamp.setPosition(.32);
        } else {
            rclamp.setPosition(0);
            lclamp.setPosition(0);
        }
    }
}
