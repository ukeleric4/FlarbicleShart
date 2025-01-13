package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@TeleOp
public class PedroTest extends LinearOpMode {
    Follower follower;
    PathChain bucketPath;
    Pose bucketPose;
    Gamepad oldGamepad1;
    Gamepad oldGamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(133.809, 86.622,0));
        bucketPose = new Pose(83, 47.6, 135);

        while (opModeIsActive()) {
            if (gamepad1.a && !oldGamepad1.a) {
                goToBucket();
                sleep(200);
            }

            follower.update();
            oldGamepad1 = gamepad1;
            oldGamepad2 = gamepad2;
        }

    }
    public void goToBucket() {
        bucketPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(bucketPose)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), bucketPose.getHeading())
                .build();
        follower.followPath(bucketPath);
    }
}
