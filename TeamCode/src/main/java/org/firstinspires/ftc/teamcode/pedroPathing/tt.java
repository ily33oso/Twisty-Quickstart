package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="trial")

public class tt extends OpMode {
    private DcMotor fl,fr,br,bl;

    @Override
        public void init(){
        fl = hardwareMap.get(DcMotor.class, "Front Left Motor");
        fr = hardwareMap.get(DcMotor.class, "Front Right Motor");
        bl = hardwareMap.get(DcMotor.class, "Back Left Motor");
        br = hardwareMap.get(DcMotor.class, "Back Right Motor");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);



    }

    @Override
    public void loop() {

    }

}
