package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Testing extends OpMode {
   Servo rclaw, lclaw;

    @Override
    public void init() {
        rclaw=hardwareMap.get(Servo.class, "rclaw");
        lclaw=hardwareMap.get(Servo.class, "lclaw");


        rclaw.setDirection(Servo.Direction.REVERSE);
        lclaw.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {

        //claw
        if (gamepad2.x){
            rclaw.setPosition(.1);
            lclaw.setPosition(.1);
        } else {
            rclaw.setPosition(0);
            lclaw.setPosition(0);
        }

    }
}
