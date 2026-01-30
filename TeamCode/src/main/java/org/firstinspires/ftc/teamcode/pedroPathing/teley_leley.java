package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp
public class teley_leley extends LinearOpMode {

    // ================= HARDWARE =================
    DcMotor fr, fl, bl, br, lift;
    DcMotorEx rwheel, lwheel;
    DcMotorEx intake;   // now DcMotorEx
    Servo rstopper, lstopper;
    IMU imu;

    // ================= DRIVE CONSTANTS =================
    final double DRIVE_POWER_MAX = 0.9;

    // ================= LIFT CONSTANTS =================
    final double TICKS_PER_MM = 48.0625;
    final double MAX_MM = 285.04551365;
    final int MAX_TICKS = (int)(TICKS_PER_MM * MAX_MM);
    final int MIN_TICKS = 0;
    final double UP_POWER = 1.0;
    final double DOWN_POWER = -0.75;

    // ================= FLYWHEEL CONSTANTS =================
    final double TICKS_PER_REV = 537.7;
    final double TARGET_RPM = 170; //215 original 190 is good 170 okay 160 not that good 165 is meh  stick with 170 for now
    final double SMALL_TARGET_RPM = 216; //250 original 225 is a bit to much power 215 really consistent with 2 artifacts
    double targetVelocity, smallTargetVelocity;

    // ================= INTAKE PID CONSTANTS =================
    // Target speed for one-ball feed / normal intake
    final double INTAKE_RPM = -800;      // ~70% of free speed, strong but not max
    final double REVERSE_RPM = 600;    // reverse to clear jams


    final double INTAKE_VELO = (INTAKE_RPM / 60.0) * TICKS_PER_REV;
    final double REVERSE_VELO = (REVERSE_RPM / 60.0) * TICKS_PER_REV;


    final long FEED_TIME_MS = 180;   // one ball
    boolean feeding = false;
    long feedStartTime = 0;
    boolean lastA = false;

    @Override
    public void runOpMode() {

        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");
        lift = hardwareMap.get(DcMotor.class, "lift");

        rwheel = hardwareMap.get(DcMotorEx.class, "rwheel");
        lwheel = hardwareMap.get(DcMotorEx.class, "lwheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");   // PID intake

        rstopper = hardwareMap.get(Servo.class, "rstopper");
        lstopper = hardwareMap.get(Servo.class, "lstopper");

        imu = hardwareMap.get(IMU.class, "imu");

        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rwheel.setDirection(DcMotorSimple.Direction.FORWARD);
        lwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        rwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rwheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        lwheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        rstopper.setDirection(Servo.Direction.REVERSE);
        lstopper.setDirection(Servo.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // PID for 1150 intake
        intake.setVelocityPIDFCoefficients(14.0, 0.0, 1.0, 14.5);


        rwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);
        lwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);

        targetVelocity = (TARGET_RPM / 60.0) * TICKS_PER_REV;
        smallTargetVelocity = (SMALL_TARGET_RPM / 60.0) * TICKS_PER_REV;

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

            // ================= FLYWHEEL =================
            if (gamepad2.dpad_down) {
                rwheel.setVelocity(targetVelocity);
                lwheel.setVelocity(targetVelocity);
            } else if (gamepad2.dpad_up) {
                rwheel.setVelocity(smallTargetVelocity);
                lwheel.setVelocity(smallTargetVelocity);
            } else if (gamepad2.left_bumper) {
                rwheel.setVelocity(0);
                lwheel.setVelocity(0);
            }

            // ================= STOPPER =================
            if (gamepad2.right_bumper) {
                rstopper.setPosition(.23);
                lstopper.setPosition(.23);
            } else {
                rstopper.setPosition(0);
                lstopper.setPosition(0);
            }

            // ================= PID INTAKE =================
            boolean a = gamepad2.a;
            boolean x = gamepad2.x;
            boolean y = gamepad2.y;

            if (a && !lastA && !feeding) {
                feeding = true;
                feedStartTime = System.currentTimeMillis();
                intake.setVelocity(INTAKE_VELO);
            }
            lastA = a;

            if (feeding && System.currentTimeMillis() - feedStartTime >= FEED_TIME_MS) {
                feeding = false;
                intake.setVelocity(0);
            }

            if (x) {
                intake.setVelocity(INTAKE_VELO);
            } else if (y) {
                intake.setVelocity(REVERSE_VELO);
            } else if (!feeding) {
                intake.setVelocity(0);
            }

            // ================= LIFT =================
            int pos = lift.getCurrentPosition();
            if (gamepad1.dpad_up && pos < MAX_TICKS) lift.setPower(UP_POWER);
            else if (gamepad1.dpad_down && pos > MIN_TICKS) lift.setPower(DOWN_POWER);
            else lift.setPower(0);

            telemetry.addData("Intake RPM", intake.getVelocity() * 60 / TICKS_PER_REV);

            telemetry.update();
        }
    }
}

