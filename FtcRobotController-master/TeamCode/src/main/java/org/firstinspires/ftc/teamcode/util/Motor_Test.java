package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.parts.Panning;
import org.firstinspires.ftc.teamcode.parts.Servos;

@TeleOp
public class Motor_Test extends OpMode {
    Panning panning;
    Servos pitching;

    @Override
    public void init() {
        panning = new Panning(hardwareMap);
        pitching = new Servos(hardwareMap, "pitching");
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            panning.runUp();
        } else if (gamepad1.a) {
            panning.runDown();
        } else {
            panning.stop();
        }

        if (gamepad1.b) {
            pitching.moveToMax();
        } else if (gamepad1.x) {
            pitching.moveToMin();
        }
    }
}
