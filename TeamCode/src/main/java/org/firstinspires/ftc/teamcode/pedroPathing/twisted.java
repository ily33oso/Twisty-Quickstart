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


@Autonomous(name = "PLS WORK SPEED", group = "robot")
public class twisted extends LinearOpMode {


    // ================= PEDRO =================
    private Follower follower;
    private Timer pathTimer;
    private boolean pathStarted = false;


    // ================= HARDWARE (SHOOTER ONLY) =================
    private DcMotorEx rwheel, lwheel;
    private DcMotorEx intake;
    private Servo rstopper, lstopper;


    // ================= POSES (FROM YOUR AUTO) =================
    private final Pose startPose = new Pose(24.128, 122.816, Math.toRadians(90));
    private final Pose p1 = new Pose(62.912, 122.784, Math.toRadians(90));
    private final Pose p2 = new Pose(58.368, 93.056, Math.toRadians(130));
    private final Pose p3 = new Pose(17.512, 84.176);
    private final Pose p4 = new Pose(48.768, 101.568, Math.toRadians(150));


    private PathChain path1, path2, path3, path4;


    // ================= TELEOP SHOOTER CONSTANTS (UNCHANGED) =================
    final double TICKS_PER_REV = 537.7;
    final double TARGET_RPM = 170;
    final double SMALL_TARGET_RPM = 216;


    double targetVelocity;
    double smallTargetVelocity;


    // Intake feed speeds
    final double INTAKE_RPM = -800;
    final double REVERSE_RPM = 600;


    final double INTAKE_VELO = (INTAKE_RPM / 60.0) * TICKS_PER_REV;
    final double REVERSE_VELO = (REVERSE_RPM / 60.0) * TICKS_PER_REV;


    final long FEED_TIME_MS = 180;


    // ================= AUTO SHOOT SEQUENCER =================
    private int shotsDone = 0;
    private double shootStartTime = 0;         // seconds
    private double lastFeedStartTime = -999;   // seconds
    private boolean feedingNow = false;


    // ================= STATES =================
    private enum State {
        PATH1,
        PATH2,
        SHOOT_3,
        PATH3,
        PATH4,
        STOP
    }


    private State state = State.PATH1;


    @Override
    public void runOpMode() {


        // ===== PEDRO INIT =====
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        pathTimer = new Timer();
        buildPaths();


        // ===== SHOOTER INIT (FROM TELEOP) =====
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


        // PID coefficients (UNCHANGED)
        rwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);
        lwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);
        intake.setVelocityPIDFCoefficients(14.0, 0.0, 1.0, 14.5);


        targetVelocity = (TARGET_RPM / 60.0) * TICKS_PER_REV;
        smallTargetVelocity = (SMALL_TARGET_RPM / 60.0) * TICKS_PER_REV;


        // Default safe positions
        stopShooter();
        intakeOff();
        closeStopper();


        telemetry.addLine("AUTO READY (Paths + Tele Shooter)");
        telemetry.update();


        waitForStart();
        if (isStopRequested()) return;


        state = State.PATH1;


        while (opModeIsActive()) {
            follower.update();
            updateStateMachine();


            telemetry.addData("State", state);
            telemetry.addData("ShotsDone", shotsDone);
            telemetry.addData("FlywheelVelR", rwheel.getVelocity());
            telemetry.addData("FlywheelVelL", lwheel.getVelocity());
            telemetry.update();
        }
    }


    // ================= PATHS (FROM YOUR AUTO) =================
    private void buildPaths() {


        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, p1))
                .setLinearHeadingInterpolation(startPose.getHeading(), p1.getHeading())
                .build();


        path2 = follower.pathBuilder()
                .addPath(new BezierLine(p1, p2))
                .setLinearHeadingInterpolation(p1.getHeading(), p2.getHeading())
                .build();


        path3 = follower.pathBuilder()
                .addPath(new BezierLine(p2, p3))
                .setTangentHeadingInterpolation()
                .build();


        path4 = follower.pathBuilder()
                .addPath(new BezierLine(p3, p4))
                .setLinearHeadingInterpolation(Math.toRadians(90), p4.getHeading())
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
                    state = State.PATH2;
                }
                break;


            case PATH2:
                if (!pathStarted) {
                    follower.followPath(path2);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;


                    // Setup shooting
                    pathTimer.resetTimer();
                    shootStartTime = 0;
                    shotsDone = 0;
                    lastFeedStartTime = -999;
                    feedingNow = false;


                    state = State.SHOOT_3;
                }
                break;


            case SHOOT_3:
                // Non-blocking shooter sequence:
                // - spin up
                // - open stopper
                // - feed 3 shots with timed intake pulses
                // - stop everything
                runShoot3Sequence();
                break;


            case PATH3:
                if (!pathStarted) {
                    follower.followPath(path3);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    state = State.PATH4;
                }
                break;


            case PATH4:
                if (!pathStarted) {
                    follower.followPath(path4);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    state = State.STOP;
                }
                break;


            case STOP:
                // Safety shutdown
                stopShooter();
                intakeOff();
                closeStopper();
                break;
        }
    }


    // ================= SHOOT 3 SEQUENCE (AUTO SAFE) =================
    private void runShoot3Sequence() {


        double t = pathTimer.getElapsedTimeSeconds();


        // Mark start
        if (shootStartTime == 0) {
            shootStartTime = t;
            startShooterHigh();   // use high shot mode (170rpm)
            openStopper();
        }


        // Simple spin-up time (tweak if needed)
        double timeSinceStart = t - shootStartTime;


        // Wait for spin-up before first feed
        if (timeSinceStart < 0.8) {
            intakeOff();
            return;
        }


        // Feeding pulse control
        double feedPulseSeconds = FEED_TIME_MS / 1000.0;


        // If currently feeding, stop after pulse duration
        if (feedingNow) {
            if (t - lastFeedStartTime >= feedPulseSeconds) {
                intakeOff();
                feedingNow = false;
            }
            return;
        }


        // If we still have shots to do, start a feed pulse every ~0.35s
        if (shotsDone < 3) {
            // gap between shots (tweak)
            double gap = 0.35;


            if (t - lastFeedStartTime >= gap) {
                intakeOn();                 // start feeding
                feedingNow = true;
                lastFeedStartTime = t;
                shotsDone++;
            }
            return;
        }


        // Done shooting: shut down and move on
        closeStopper();
        intakeOff();
        stopShooter();


        state = State.PATH3;
    }


    // ================= TELEOP SHOOTER METHODS (EXTRACTED) =================
    private void startShooterHigh() {
        rwheel.setVelocity(targetVelocity);
        lwheel.setVelocity(targetVelocity);
    }


    @SuppressWarnings("unused")
    private void startShooterLow() {
        rwheel.setVelocity(smallTargetVelocity);
        lwheel.setVelocity(smallTargetVelocity);
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


    @SuppressWarnings("unused")
    private void intakeReverse() {
        intake.setVelocity(REVERSE_VELO);
    }


    private void intakeOff() {
        intake.setVelocity(0);
    }
}
