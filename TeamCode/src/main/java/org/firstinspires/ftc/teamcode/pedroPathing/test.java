package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class test extends OpMode {
    DcMotor fr, fl, br, bl, rintake, lintake;
    Servo rclamp, lclamp, rclaw, lclaw, arm;

    @Override
    public void init() {
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");

        rintake = hardwareMap.get(DcMotor.class, "rintake");
        lintake = hardwareMap.get(DcMotor.class, "lintake");

        rclamp = hardwareMap.get(Servo.class, "rclamp");
        lclamp = hardwareMap.get(Servo.class, "lclamp");

        rclaw=hardwareMap.get(Servo.class, "rclaw");
        lclaw=hardwareMap.get(Servo.class, "lclaw");

        arm =hardwareMap.get(Servo.class, "arm");

        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);

        rintake.setDirection(DcMotor.Direction.FORWARD);
        lintake.setDirection(DcMotor.Direction.REVERSE);

        rclamp.setDirection(Servo.Direction.REVERSE);
        lclamp.setDirection(Servo.Direction.FORWARD);

        rclaw.setDirection(Servo.Direction.REVERSE);
        lclaw.setDirection(Servo.Direction.FORWARD);

        arm.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        // ================= CLAMP =================
        if (gamepad1.left_bumper){
            rclamp.setPosition(.32);
            lclamp.setPosition(.32);
        } else {
            rclamp.setPosition(0);
            lclamp.setPosition(0);
        }


        // ================= INTAKE =================
        if (gamepad2.a) {
            lintake.setPower(.5);
            rintake.setPower(.5);
        } else {
            lintake.setPower(0);
            rintake.setPower(0);
        }

        // ================= claw =================
        if (gamepad2.b){
            rclaw.setPosition(.1);
            lclaw.setPosition(.1);
        } else{
            rclaw.setPosition(0);
            lclaw.setPosition(0);
        }

        // ================= arm =================
        if (gamepad2.right_bumper){
            arm.setPosition(1);
        } else{
            arm.setPosition(0);
        }



    }
}
