package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.PIDFPanning;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.Servos;

import org.firstinspires.ftc.teamcode.parts.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@TeleOp(name="PedroDriverControl", group="Linear Opmode")  //@Autonomous(...) is the other common choice
public class PedroDriver extends LinearOpMode {
    public PIDFPanning panningMotor;
    public Claw claw;
    public Servos orientation;
    public Servos panningServo;
    public Servos pitching;
    public PIDFSlide slides;
    public Vision vision;

    // gamepad old for button
    Gamepad oldGamepad1;
    Gamepad oldGamepad2;

    // random
    private double velocity;
    private boolean usePID = true;

    // auto to bucket stuff
    private Follower follower;
    private Pose currentPose;
    private PathChain toBucket, toSubmersible;
    private Pose bucketPose, subPose;
    private PathBuilder builderBucket;
    private PathChain builderSub;
    int bucketCase = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        bucketPose = new Pose(83, 47.6, 135);
        slides = new PIDFSlide(hardwareMap);
        panningMotor = new PIDFPanning(hardwareMap);
        claw = new Claw(hardwareMap);
        orientation = new Servos(hardwareMap, "orientation");
        panningServo = new Servos(hardwareMap, "panning");
        pitching = new Servos(hardwareMap, "pitching");
        follower = new Follower(hardwareMap);
        vision = new Vision(hardwareMap, telemetry);

        orientation.moveToMin();
        follower.startTeleopDrive();
        follower.setStartingPose(new Pose(133.809, 86.622,0));

        while (opModeIsActive()) {
            // Speed control using triggers
            updateVelocity();
            // Slide control using gamepad1 bumpers
            manualSlide();
            // Pitching
            pitching();
            // Panning on gamepad 2
            manualPanning();
            // Ai orientation
            cvOrientation();
            // Manual panning servo
            manualPanServo();
            // Claw on game pad 2 + pitching does down + panning servo
            clawMovement();
            // Bring panning up and make slide to go bucket pos
            // Bring slide down and panning down
            bucketPos();
            // Automatically run slide until it sees a block
            autoSlide();
            // Update pedro pathing follower, cv, panning, slide
            update();

//           manualPanServo();
//            clawMovement();
//            cvOrientation();
//            autoSlide();
//            manualOrientation();
//            slidePosition();
//            manualPanning();
//            bucketCaseChange();
//            updateFollower();

//             runToBucket(bucketCase);

            oldGamepad1 = gamepad1;
            oldGamepad2 = gamepad2;
        }
    }

    public void runToBucket(int num) {
        switch (num) {
            case 1:
                follower.breakFollowing();
                follower.setMaxPower(0.65);

                toBucket = builderBucket
                        .addPath(new BezierCurve(new Point(follower.getPose()), new Point(bucketPose)))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), bucketPose.getHeading())
                        .build();

                follower.followPath(toBucket, true);
                follower.update();
                movePanningUp();
                slides.setTargetPos(2000);
                bucketCase = 2;
                break;
            case 2:
                if (!follower.isBusy()) {
                    bucketDrop();
                    slides.setTargetPos(0);
                }
                break;
        }
    }
    // Teleop functions
    public void updateVelocity() {
        if (gamepad1.right_trigger > 0.8) {
            velocity = 1;
        } else if (gamepad1.left_trigger > 0.8) {
            velocity = 0.25;
        } else {
            velocity = 0.6;
        }
    }

    public void pitching() {
        if (gamepad1.a) {
            pitching.moveToMin();
        } else if (gamepad1.y) {
            pitching.moveToMax();
        }
    }

    public void cvOrientation() {
        double pos = vision.getOrientation();
        if (gamepad1.b) {
            if (pos != 0.01) {
                orientation.moveSpecificPos(pos);
            }
        }
    }

    public void manualPanServo() {
        if (gamepad2.y) {
            panningServo.moveSpecificPos(0.3);  //Move up
        } else if (gamepad2.a) {
            panningServo.moveToMax();  //Move down
        }
    }

    public void bucketPos() {
        if (gamepad2.x) {
            movePanningUp();
            usePID = true;
            slides.setTargetPos(2000);
        } else if (gamepad2.b) {
            slides.setTargetPos(0);
            if (slides.getCurrentPos() < 1000) {
                movePanningDown();
            }
        }
    }

    public void manualSlide() {
        if (gamepad1.right_bumper) {
            usePID = false;
            slides.setPower(1);
        } else if (gamepad1.left_bumper) {
            usePID = false;
            slides.setPower(-1);
        } else {
            usePID = true;
        }
    }

    public void manualPanning() {
        if (gamepad2.right_trigger > 0.8) {
           movePanningUp();
        } else if (gamepad2.left_trigger > 0.8) {
            movePanningDown();
        } else {
            keepPanning();
        }
    }

    public void clawMovement() {
        if (gamepad2.dpad_left) {
            pitching.moveToMin();
            claw.closeClaw();
            sleep(250);
            panningServo.moveSpecificPos(0.3);
        } else if (gamepad2.dpad_right) {
            claw.openClaw();
        }
    }

    public void autoSlide() {
        if (gamepad2.dpad_up) {
            while (vision.getFirstBlock() == null) {
                usePID = false;
                slides.setPower(0.5);
            }
        }
    }

    public void bucketCaseChange() {

    }

    public void update() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity , -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * velocity);
        follower.update();
        currentPose = follower.getPose();

        vision.updateVision();
        slides.updateSlide();
        slides.setTargetPos(slides.getCurrentPos());
        if (usePID) {
            slides.updatePower();
        }

        telemetry.addData("slide encoder pos:", slides.getCurrentPos());
    }

    // Other functions
    public void movePanningDown() {
        panningMotor.setPower(-1);
    }
    public void movePanningUp() {
        panningMotor.setPower(1);
    }

    public void keepPanning() {
        panningMotor.setPower(0);
        panningMotor.setModeBrake();
    }

    public void bucketDrop() {
        panningServo.moveSpecificPos(0.35);
        sleep(200);
        claw.openClaw();
        sleep(200);
        panningServo.moveSpecificPos(0.7);
        sleep(200);
    }
}

