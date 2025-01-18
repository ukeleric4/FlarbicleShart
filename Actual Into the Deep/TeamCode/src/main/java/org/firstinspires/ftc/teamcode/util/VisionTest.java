package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.Vision;
import org.firstinspires.ftc.teamcode.parts.Servos;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Disabled
@TeleOp
public class VisionTest extends LinearOpMode {
    Vision vision;
    Servos orientation;

    double pos;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        vision = new Vision(hardwareMap, telemetry);
        orientation = new Servos(hardwareMap, "orientation");


        while (opModeIsActive()) {
            if (gamepad1.x) {
                orientation.moveToMin();
            } else if (gamepad1.b) {
                orientation.moveToMax();
            }

            pos = vision.getOrientation();

            if (pos != 0.01) {
                orientation.moveSpecificPos(vision.getOrientation());
            }

            vision.updateVision();
        }
    }
}
