package org.firstinspires.ftc.teamcode.autonomous;


import static java.lang.Thread.sleep;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.PIDFPanning;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.Panning;
import org.firstinspires.ftc.teamcode.parts.Servos;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "SUBMERSIBLE", group = "Game Auto")
public class Submersible extends OpMode {

    /* ----------AARRUSH NOTES---------- */
    /*
        The code labeled "Poses" does NOT have updated poses.
        What we fixed on Thursday is not reflected in them.
        When coding the auto, a path such as "hangFirst" should be used, instead of a
            path copied from the online path maker.
        If there is a need to change/tweak a path, you should ONLY tweak the poses
            related to that path. This way you should only have to change the poses,
            as the changes will be made automatically in the buildPaths() function,
            since it uses the poses to create paths, rather than concrete numbers.
        As a reminder:
            1. DO NOT USE THE PATHS MADE THE ONLINE PATH BUILDER.
            2. DO NOT USE THE PATH LABELED: 'fullPath' unless for testing purposes

        In this way, the process of making a case is as follows:
            1. Test a single path. There are a total of 16 paths, each corresponding to
                 a certain action. If the path you are testing (such as the one that moves
                 the robot to the submersible) needs to be tweaked, change the poses that correspond
                 to said path. To make it easier, the poses are labeled under which path
                 they correspond to.
            2. Once the path is correct, or you cannot test if it works without motor code,
                you can begin writing motor code. The slide is controlled with slides.setTargetPos(int pos).
                The maximum the slide can extend to is around 1750, but an extension of 600 combined with
                panning should do the job for hanging the specimens on the bar. For panning,
                use panning.runUp/panning.runDown/panning.stop . Keep in mind that if you run:
                panning.runUp(), the panning will continue to run unless you run panning.stop().
                If you want, you can also look at the class (its simple) and add a method to run
                the panning for time.
            3. Finally, run setPathState() in order to change the case to the next one.
                Keep in mind: Once you switch to the next case, you MUST check for something
                (such as follower.isBusy() or follower.getX > 130) in order to prevent
                constant looping and multiple threads running.
            4. When you wish to end the auto, run setPathState(-1), as this will set the case
                to an unreachable value, essentially ending the auto. This should also be used
                when testing the full path to prevent it from being run multiple times.
            5. In order to test the full path, comment out all cases and only keep the following
                lines of code:
                    follower.followPath(fullPath);
                    setPathState(-1);
                This will make the robot follow the entire path, as well as make the case
                an arbitrary number, making it so there are no other instructions after
                you follow the full path.

            If you have any questions just message me since there are some weird things like holdEnd

            If you want to figure out what holdEnd is (self-explanatory), look at the pedro pathing
            docs. It may come in useful when grabbing blocks off the wall or off the ground.

            - Good luck

     */

    // Parts
    public Panning panning;
    public Claw claw;
    public Servos orientation;
    public Servos pitching;
    public Servos panningServo;
    public PIDFSlide slides;

    // Follower stuff
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    /* Poses */
    private final Pose startPose = new Pose(135, 80, 0); // line 1
    private final Pose hangPose1 = new Pose(106, 78, 0); // line 1
    private final Pose pickUpPose1 = new Pose(106, 107, 120); // line 2
    private final Pose dropOffPose1 = new Pose(115, 112, 40); // line 3
    private final Pose pickUpPose2 = new Pose(102, 117, 105); // line 4
    private final Pose dropOffPose2 = new Pose(115, 125, 45); // line 5
    private final Pose pickUpPose3 = new Pose(106, 131, 125); // line 6
    private final Pose grab1 = new Pose(125.5, 110.5, 0); // line 7
    private final Pose grab = new Pose(125.5, 110.5, 0); // lines 9, 11, 13
    private final Pose hangPose2 = new Pose(104.5, 76, 0); // line 8
    private final Pose hangPose3 = new Pose(104.5, 74, 0); // line 10
    private final Pose hangPose4 = new Pose(104.5, 72, 0); // line 12
    private final Pose hangPose5 = new Pose(104.5, 70, 0); // line 14
    private final Pose park = new Pose(122, 97, 60); // line 15

    /* Control Points */
    private final Point pickUpPose1Control = new Point(119, 80);
    private final Point grab1Control = new Point(122, 129);
    private final Point hangControl = new Point(124, 69);
    private final Point grabControl = new Point(115, 80);


    // Path chains to be used in buildPaths()
    private PathChain hangFirst, grabFirst, dropOffFirst, grabSecond,
            dropOffSecond, grabThird, grabSpecSecond, hangSpecSecond,
            grabSpecThird, hangSpecThird, grabSpecFourth, hangSpecFourth,
            grabSpecFifth, hangSpecFifth, endPark, fullPath;


    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                /*
                 REMINDER:
                    This is the code to run the entire path. Comment everything else out first to test.
                        follower.followPath(fullPath);
                        setPathState(-1);
                    In order to change the path: 'fullPath'
                        1. Scroll down to buildPaths() and find 'fullPath'
                        2. Replace everything from .addPath to just before .build()
                        3. Copy in paste code that starts with .addPath() from the online
                            path maker.
                        4. If there is in error, its likely that you either forgot a semicolon
                            or that you forgot to add .build() after everything else

                    EXAMPLE CASE
                    This code should be able to hang the preloaded specimen on the bar.
                    Here is a detailed breakdown of what it does:
                        1. The follower is told to follow the path 'hangFirst'
                            It will begin to do this when follower.update() is called.
                        2. The panning is told to run up as far is can
                        3. The slides are run to position 600 (may have to be tweaked)
                            which should be high enough to be level with the bar
                        4. The servo is run to a position where it is holding the specimen
                            where it is perfectly horizontal and the clip is horizontally facing
                            the bar
                        5. The case is changed to 1
                    SCORING MECHANISM
                    - This is a passive scoring mechanism as if the robot drives backwards enough
                        the clip will be clipped onto the bar (no servo movement required)
                 */
                follower.followPath(hangFirst);
                panning.runUp();
                slides.setTargetPos(600);
                movePanServoScore();
                setPathState(1);
            case 1:
                /*
                    EXAMPLE CASE CONT.
                    1. When the follower is no longer busy (finished following path),
                        it runs the following code
                    2. It begins to follow the path meant to bring the robot to the position
                        where it can pick up the first block by extending its slide
                    3. The claw opens since the specimen is already clipped on
                    4. The slides are brought a bit back to a position that should be tweaked
                        so that the claw is over the block (orientation is also tweaked)
                    5. Panning runs downwards
                    6. Path case is set to '2'

                    FINAL REMINDERS:
                    - If you find that follower.isBusy() is too slow, you should consider either checking
                        if the robot is above a certain threshold in either X or Y:
                            follower.getPose().getX() > 160; // Or some other value depending on the pose it should reach
                            follower.getPose().getX() > hangPose1.getX() - 5;
                            ^ Checks if the current pose's X is 5 inches less than the pose it needs to reach
                 */
                if (!follower.isBusy()) {
                    follower.followPath(grabFirst);
                    claw.openClaw();
                    orientation.moveSpecificPos(0.3);
                    slides.setTargetPos(500);
                    panning.runDown();
                    setPathState(2);
                }
                break;
            case 2:
                // You can start writing code to pick up the first block here:
                // Don't forget to include a 'break;' at the end of every case.
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    // Runs when "init" is pressed
    @Override
    public void init() {
        // Get things from hardware map
        slides = new PIDFSlide(hardwareMap);
        panning = new Panning(hardwareMap);
        claw = new Claw(hardwareMap);
        pitching = new Servos(hardwareMap, "pitching");
        orientation = new Servos(hardwareMap, "orientation");
        panningServo = new Servos(hardwareMap, "panning");

        // Make timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Set up follower
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // Place parts in initial positions
        claw.closeClaw(); // Closed claw
        panningServo.moveToMin(); // Panning servo UP
        pitching.moveToMin(); // Pitching UP
        orientation.moveToMin(); // Orientation at normal pos
        slides.setTargetPos(20); // Slides fully retracted
    }

    // Runs when "start" is pressed
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    // Stop method to stop the opMode
    @Override
    public void stop() {}

    // Helper functions

    public void movePanServoScore() { panningServo.moveSpecificPos(0.2); }
    public void movePanServoGrab() { panningServo.moveSpecificPos(0.35); }
    public void movePanServoReady() {panningServo.moveSpecificPos(0.2);}

    // Builds all paths prior to running the auto (init)
    public void buildPaths() {
        hangFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(hangPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        grabFirst = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hangPose1), pickUpPose1Control, new Point(pickUpPose1)))
                .setLinearHeadingInterpolation(hangPose1.getHeading(), pickUpPose1.getHeading())
                .build();
        dropOffFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose1), new Point(dropOffPose1)))
                .setLinearHeadingInterpolation(pickUpPose1.getHeading(), dropOffPose1.getHeading())
                .build();
        grabSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropOffPose1), new Point(pickUpPose2)))
                .setLinearHeadingInterpolation(dropOffPose1.getHeading(), pickUpPose2.getHeading())
                .build();
        dropOffSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose2), new Point(dropOffPose2)))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), dropOffPose2.getHeading())
                .build();
        grabThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropOffPose2), new Point(pickUpPose3)))
                .setLinearHeadingInterpolation(dropOffPose2.getHeading(), pickUpPose3.getHeading())
                .build();
        grabSpecSecond = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickUpPose3), grab1Control, new Point(grab1)))
                .setLinearHeadingInterpolation(pickUpPose3.getHeading(), grab1.getHeading())
                .build();
        hangSpecSecond = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab1), hangControl, new Point(hangPose2)))
                .setConstantHeadingInterpolation(0)
                .build();
        grabSpecThird = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hangPose2), grabControl, new Point(grab)))
                .setConstantHeadingInterpolation(0)
                .build();
        hangSpecThird = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab), hangControl, new Point(hangPose3)))
                .setConstantHeadingInterpolation(0)
                .build();
        grabSpecFourth = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hangPose3), grabControl, new Point(grab)))
                .setConstantHeadingInterpolation(0)
                .build();
        hangSpecFourth = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab), hangControl, new Point(hangPose4)))
                .setConstantHeadingInterpolation(0)
                .build();
        grabSpecFifth = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hangPose4), grabControl, new Point(grab)))
                .setConstantHeadingInterpolation(0)
                .build();
        hangSpecFifth = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab), hangControl, new Point(hangPose5)))
                .setConstantHeadingInterpolation(0)
                .build();
        endPark = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hangPose5), new Point(park)))
                .setLinearHeadingInterpolation(hangPose5.getHeading(), park.getHeading())
                .build();

        fullPath = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(134.917, 79.975, Point.CARTESIAN),
                                new Point(105.888, 79.931, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(105.888, 79.931, Point.CARTESIAN),
                                new Point(119.279, 80.137, Point.CARTESIAN),
                                new Point(101.562, 105.064, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(101.562, 105.064, Point.CARTESIAN),
                                new Point(115.365, 111.657, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(40))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(115.365, 111.657, Point.CARTESIAN),
                                new Point(100.738, 116.395, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(105))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(100.738, 116.395, Point.CARTESIAN),
                                new Point(115.365, 125.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(45))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(115.365, 125.000, Point.CARTESIAN),
                                new Point(100.326, 128.549, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(125))
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(100.326, 128.549, Point.CARTESIAN),
                                new Point(121.545, 129.373, Point.CARTESIAN),
                                new Point(125.459, 110.627, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(0))
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(125.459, 110.627, Point.CARTESIAN),
                                new Point(124.017, 69.219, Point.CARTESIAN),
                                new Point(105.682, 79.931, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(105.682, 79.931, Point.CARTESIAN),
                                new Point(115.000, 80.000, Point.CARTESIAN),
                                new Point(125.459, 110.627, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(125.459, 110.627, Point.CARTESIAN),
                                new Point(124.017, 69.219, Point.CARTESIAN),
                                new Point(105.682, 78.695, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(105.682, 78.695, Point.CARTESIAN),
                                new Point(115.000, 80.000, Point.CARTESIAN),
                                new Point(125.459, 110.627, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 12
                        new BezierCurve(
                                new Point(125.459, 110.627, Point.CARTESIAN),
                                new Point(124.017, 69.219, Point.CARTESIAN),
                                new Point(105.476, 77.253, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 13
                        new BezierCurve(
                                new Point(105.476, 77.253, Point.CARTESIAN),
                                new Point(115.000, 80.000, Point.CARTESIAN),
                                new Point(125.459, 110.833, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 14
                        new BezierCurve(
                                new Point(125.459, 110.833, Point.CARTESIAN),
                                new Point(124.017, 69.219, Point.CARTESIAN),
                                new Point(105.476, 74.987, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 15
                        new BezierLine(
                                new Point(105.476, 74.987, Point.CARTESIAN),
                                new Point(121.957, 96.618, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60)).build();

    }
}