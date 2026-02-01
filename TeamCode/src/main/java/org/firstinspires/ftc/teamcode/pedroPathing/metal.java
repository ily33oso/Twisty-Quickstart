package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// ===== PEDRO =====
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

// ===== HARDWARE =====
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "PLS WORK SPEED (RED)", group = "robot")
public class metal extends LinearOpMode {

    // ================= PEDRO =================
    private Follower follower;
    private Timer pathTimer;
    private boolean pathStarted = false;

    // ================= MECH =================
    private DcMotorEx rwheel, lwheel;
    private DcMotorEx intake;
    private Servo rstopper, lstopper;

    // ================= POSES (RED SIDE) =================
    // Blue → Red mirror: y = 144 - y, heading = -heading
    private final Pose startPose = new Pose(22.976, 18.496, Math.toRadians(40));
    private final Pose p1        = new Pose(59, 55, Math.toRadians(40));
    private final Pose p2        = new Pose(48.552, 67.8);

    // ================= PATHS =================
    private PathChain path1, path2;

    // ================= SHOOTER CONSTANTS =================
    final double TICKS_PER_REV = 537.7;
    final double TARGET_RPM = 170;
    final double INTAKE_RPM = -800;
    final double REVERSE_RPM = 600;
    final long FEED_TIME_MS = 180;

    double targetVelocity;
    final double INTAKE_VELO = (INTAKE_RPM / 60.0) * TICKS_PER_REV;
    final double REVERSE_VELO = (REVERSE_RPM / 60.0) * TICKS_PER_REV;

    // ================= SHOOT SEQUENCER =================
    private int shotsDone = 0;
    private double shootStartTime = -1;
    private double lastFeedStartTime = -999;
    private boolean feedingNow = false;

    // ================= STATES =================
    private enum State {
        PATH1,
        SHOOT_3,
        PATH2,
        STOP
    }

    private State state = State.PATH1;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        pathTimer = new Timer();
        buildPaths();

        initShooterMech();

        telemetry.addLine("READY (RED)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            follower.update();
            updateStateMachine();

            telemetry.addData("State", state);
            telemetry.addData("ShotsDone", shotsDone);
            telemetry.addData("Flywheel R", rwheel.getVelocity());
            telemetry.addData("Flywheel L", lwheel.getVelocity());
            telemetry.update();
        }
    }

    // ================= PATH BUILDING =================
    private void buildPaths() {

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, p1))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        p1.getHeading()
                )
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(p1, p2))
                .setTangentHeadingInterpolation()
                .build();
    }

    // ================= STATE MACHINE =================
    private void updateStateMachine() {
        switch (state) {

            case PATH1:
                if (!pathStarted) {
                    follower.followPath(path1);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    resetShootSequencer();
                    pathTimer.resetTimer();
                    state = State.SHOOT_3;
                }
                break;

            case SHOOT_3:
                runShoot3Sequence();
                break;

            case PATH2:
                if (!pathStarted) {
                    follower.followPath(path2);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    state = State.STOP;
                }
                break;

            case STOP:
                stopShooter();
                intakeOff();
                closeStopper();
                break;
        }
    }

    // ================= SHOOT SEQUENCE =================
    private void resetShootSequencer() {
        shotsDone = 0;
        shootStartTime = -1;
        lastFeedStartTime = -999;
        feedingNow = false;
    }

    private void runShoot3Sequence() {

        double t = pathTimer.getElapsedTimeSeconds();

        if (shootStartTime < 0) {
            shootStartTime = t;
            startShooterHigh();
            openStopper();
        }

        if ((t - shootStartTime) < 0.8) {
            intakeOff();
            return;
        }

        double feedPulseSeconds = FEED_TIME_MS / 1000.0;

        if (feedingNow) {
            if (t - lastFeedStartTime >= feedPulseSeconds) {
                intakeOff();
                feedingNow = false;
            }
            return;
        }

        if (shotsDone < 3) {
            if (t - lastFeedStartTime >= 0.35) {
                intakeOn();
                feedingNow = true;
                lastFeedStartTime = t;
                shotsDone++;
            }
            return;
        }

        closeStopper();
        intakeOff();
        stopShooter();
        state = State.PATH2;
    }

    // ================= MECH INIT =================
    private void initShooterMech() {

        rwheel = hardwareMap.get(DcMotorEx.class, "rwheel");
        lwheel = hardwareMap.get(DcMotorEx.class, "lwheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        rstopper = hardwareMap.get(Servo.class, "rstopper");
        lstopper = hardwareMap.get(Servo.class, "lstopper");

        rwheel.setDirection(DcMotorSimple.Direction.FORWARD);
        lwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        rwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rstopper.setDirection(Servo.Direction.REVERSE);
        lstopper.setDirection(Servo.Direction.FORWARD);

        rwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);
        lwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);
        intake.setVelocityPIDFCoefficients(14, 0, 1, 14.5);

        targetVelocity = (TARGET_RPM / 60.0) * TICKS_PER_REV;

        stopShooter();
        intakeOff();
        closeStopper();
    }

    // ================= HELPERS =================
    private void startShooterHigh() {
        rwheel.setVelocity(targetVelocity);
        lwheel.setVelocity(targetVelocity);
    }

    private void stopShooter() {
        rwheel.setVelocity(0);
        lwheel.setVelocity(0);
    }

    private void openStopper() {
        rstopper.setPosition(0.23);
        lstopper.setPosition(0.23);
    }

    private void closeStopper() {
        rstopper.setPosition(0);
        lstopper.setPosition(0);
    }

    private void intakeOn() {
        intake.setVelocity(INTAKE_VELO);
    }

    private void intakeOff() {
        intake.setVelocity(0);
    }
}
