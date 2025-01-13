package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.PIDFSlide;

@TeleOp
public class SlideTest extends LinearOpMode {
    PIDFSlide slide;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        slide = new PIDFSlide(hardwareMap);

        while (opModeIsActive()) {
            slide.updateSlide();
            if (gamepad1.right_bumper) {
                slide.setTargetPos(1000);
            }
            if (gamepad1.left_bumper) {
                slide.setTargetPos(0);
            }
        }
    }
}
