package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.Light;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.Panning;
import org.firstinspires.ftc.teamcode.parts.Servos;
import org.firstinspires.ftc.teamcode.parts.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@TeleOp(name="Submersible Teleop", group="Linear Opmode")  //@Autonomous(...) is the other common choice
public class submersible_teleop extends LinearOpMode {
    public Panning panningMotor;
    public Claw claw;
    public Servos orientation;
    public Servos panningServo;
    public Servos pitching;
    public Light light;
    public PIDFSlide slides;
    public Vision vision;
    public DistanceSensor dSensor;

    // gamepad old for button
    Gamepad oldGamepad1;
    Gamepad oldGamepad2;

    // random
    private double velocity;

    // auto to bucket stuff
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        slides = new PIDFSlide(hardwareMap);
        panningMotor = new Panning(hardwareMap);
        claw = new Claw(hardwareMap);
        orientation = new Servos(hardwareMap, "orientation");
        panningServo = new Servos(hardwareMap, "panning");
        pitching = new Servos(hardwareMap, "pitching");
        dSensor = hardwareMap.get(DistanceSensor.class, "distance");
        light = new Light(hardwareMap);
        follower = new Follower(hardwareMap);
        vision = new Vision(hardwareMap, telemetry);

        orientation.moveToMax();
        pitching.moveToMax();
        follower.startTeleopDrive();
        follower.setStartingPose(new Pose(133.809, 86.622,0));

        while (opModeIsActive()) {
            // Speed control using triggers
            updateVelocity();
            // Pitching
//            pitching();
            // Orientation
            orientation();
            // Hanging auto
            hanging();
            // Panning on gamepad 2
            manualPanning();
            // Ai orientation
            /* cvOrientation(); */
            // Manual panning servo
            manualPanServo();
            // Claw on game pad 2 + pitching does down + panning servo
            clawMovement();
            // Bring panning up and make slide to go bucket pos
            // Bring slide down and panning down
            slidePos();
            // Update distance sensor
            updateDistanceSensor();
            // Update pedro pathing follower, cv, panning, slide
            update();

            oldGamepad1 = gamepad1;
            oldGamepad2 = gamepad2;

            telemetry.addData("current pos: ", slides.getCurrentPos());
            telemetry.addData("current target: ", slides.getTargetPos());
            telemetry.update();
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

    // Test function
    public void pitching() {
        if (gamepad1.dpad_down) {
            pitching.moveToMax();
        } else if (gamepad1.dpad_up) {
            pitching.moveToMin();
        }
    }

    public void hanging() {
        if (gamepad2.dpad_left) {
            // Start hanging
            slides.setTargetPos(1500);
            while (slides.getCurrentPos() < 1450) {
                slides.updateSlide();
                slides.updatePower();
            }
            slides.setPower(-1);
            while (slides.getCurrentPos() > 625) {
                slides.updateSlide();
            }
            slides.setPower(0);
            sleep(50000);
//            slides.setPower(0);
//            panningMotor.runDown();
//            slides.setTargetPos(1500);
//            while (slides.getCurrentPos() < 1450) {
//                slides.updateSlide();
//                slides.updatePower();
//            }
        }
    }

    // Not needed for now
//    public void cvOrientation() {
//        double pos = vision.getOrientation();
//        if (gamepad1.b) {
//            if (pos != 0.01) {
//                orientation.moveSpecificPos(pos);
//            }
//        }
//    }

    // Test function
    public void manualPanServo() {
        if (gamepad2.y) {
            panningServo.moveSpecificPos(0.3);  //Move up
        } else if (gamepad2.a) {
            panningServo.moveToMax();  //Move down
        }
    }

    public void slidePos() {
        if (gamepad1.x) {
            panningMotor.runUp();
            double runtime = getRuntime();
            while (getRuntime() - runtime < 0.75) {}
            panningMotor.stop();
            slides.setTargetPos(800);
        } else if (gamepad1.b) {
            slides.setTargetPos(0);
            while (slides.getCurrentPos() > 700) {
                slides.updateSlide();
            }
            claw.openClaw();
            while (slides.getCurrentPos() > 200) {
                slides.updateSlide();
            }
            panningMotor.runDown();
            double runtime = getRuntime();
            while (getRuntime() - runtime < 0.75) {slides.updateSlide();}
            panningMotor.stop();
        } else if (gamepad1.y) {
            slides.setTargetPos(1000);
        } else if (gamepad1.a) {
            slides.setTargetPos(0);
        }
    }

    public void updateDistanceSensor() {
        double distance = dSensor.getDistance(DistanceUnit.CM);
        if (distance < 2.5 && slides.getCurrentPos() > 1600) {
            light.goToBlue();
        } else if (distance > 2.5 && slides.getCurrentPos() < 900 || slides.getCurrentPos() > 1100) {
            light.goToRed();
        } else if (distance < 2.5) {
            light.goToGreen();
        } else {
            light.goToRed();
        }
    }

    // Test function
    public void manualPanning() {
        if (gamepad2.right_trigger > 0.8) {
           panningMotor.runUp();
        } else if (gamepad2.left_trigger > 0.8) {
            panningMotor.runDown();
        } else {
            panningMotor.stop();
        }
    }

    public void clawMovement() {
        if (gamepad2.dpad_down) {
            pitching.moveToMax();
            double runtime = getRuntime();
            while (getRuntime() - runtime < 0.3) {}
            claw.closeClaw();
            double runtime2 = getRuntime();
            while (getRuntime() - runtime2 < 0.6) {}
            pitching.moveToMin();
            if (dSensor.getDistance(DistanceUnit.CM) < 2.5) {
                panningServo.moveSpecificPos(0.3);
                slides.setTargetPos(50);
            } else {
                claw.openClaw();
            }
        } else if (gamepad2.dpad_up) {
            slides.setTargetPos(1000);
            while (slides.getCurrentPos() < 950) {slides.updateSlide();}
            double runtime = getRuntime();
            while (getRuntime() - runtime < 0.3) {}
            claw.openClaw();
            slides.setTargetPos(30);
            while (slides.getCurrentPos() > 30) {slides.updateSlide();}
            panningServo.moveSpecificPos(0.3);
        }
    }

    public void orientation() {
        if (gamepad2.b) {
            orientation.moveToMax();
        }
        if (gamepad2.x) {
            orientation.moveSpecificPos(0.3);
        }
    }


    public void update() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity , -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * velocity);
        follower.update();

        vision.updateVision();

        slides.updateSlide();
        slides.updatePower();

        telemetry.addData("slide encoder pos:", slides.getCurrentPos());
    }
}

