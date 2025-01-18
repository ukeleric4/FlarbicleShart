package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.Panning;
import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.Servos;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous
public class Bucket extends LinearOpMode {
    private PIDFSlide slides;
    private Panning panning;
    private Claw claw;
    private Servos orientation;
    private Servos panningServo;

    private PathChain bucketPath;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        slides = new PIDFSlide(hardwareMap);
        panning = new Panning(hardwareMap);
        claw = new Claw(hardwareMap);
        orientation = new Servos(hardwareMap, "orientation");
        panningServo = new Servos(hardwareMap, "panning");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(134.3175965665236, 33.99141630901288, 90));
        buildPaths();

        while (opModeIsActive()) {
            if (pathState == -1) {
                setPathState(0);
            }
            follower.update();
            updatePath();
            slides.updateSlide();
            slides.updatePower();
        }
    }

    public void updatePath() {
        switch (pathState) {
            case 0:
                follower.followPath(bucketPath);
                setPathState(-2);
                break;
        }
    }

    public void buildPaths() {
        bucketPath = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(134.318, 33.991, Point.CARTESIAN),
                                new Point(126.000, 17.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(126.000, 17.000, Point.CARTESIAN),
                                new Point(117.500, 23.691, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(117.500, 23.691, Point.CARTESIAN),
                                new Point(126.000, 17.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(126.000, 17.000, Point.CARTESIAN),
                                new Point(117.500, 12.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(117.500, 12.500, Point.CARTESIAN),
                                new Point(126.000, 17.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(126.000, 17.000, Point.CARTESIAN),
                                new Point(117.500, 13.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(215))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(117.500, 13.000, Point.CARTESIAN),
                                new Point(126.000, 17.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(215), Math.toRadians(135)).build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
