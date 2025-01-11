package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.Servos;

@Disabled
@Config
@TeleOp
public class OrientationTest extends LinearOpMode {
    Servos orientation;

    public static double position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        orientation = new Servos(hardwareMap, "orientation");

        while (opModeIsActive()) {
            orientation.moveSpecificPos(position);
        }
    }
}
