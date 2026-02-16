package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous(name = "Example Auto", group = "Blue")
public class autoFar extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;


    private final Pose startPose = new Pose(57.88372093023256, 8.46511627906977, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose GrabPlayerZone = new Pose(10.325581395348829, 8.46511627906977, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose ScorePlayerZone = new Pose(58.527906976744184, 15.406976744186043, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose GrabLastChain = new Pose(21.46511627906974, 37.27906976744186, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose ControlToGrabLastChain = new Pose(55.1, 37.8); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose ScoreLastChain = new Pose(49.3953488372093, 8.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose GrabPlayerZone2 = new Pose(10.3, 8.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose ScorePlayerZone2 = new Pose(49.4, 8.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
//    private Path scorePreload;
    private Path grabPlayer;
    private PathChain ScorePlayer, GrabLast, ScoreLast, GrabPlayer2, Scoreplayer2;

    public void buildPaths() {
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(startPose, GrabPlayerZone));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), GrabPlayerZone.getHeading());
        grabPlayer = new Path(new BezierLine(startPose, GrabPlayerZone));
        grabPlayer.setLinearHeadingInterpolation(startPose.getHeading(), GrabPlayerZone.getHeading());
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        ScorePlayer = follower.pathBuilder()
                .addPath(new BezierLine(GrabPlayerZone, ScorePlayerZone))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        GrabLast = follower.pathBuilder()
                .addPath(new BezierCurve(ScorePlayerZone, ControlToGrabLastChain,GrabLastChain))
                .setTangentHeadingInterpolation()
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        ScoreLast = follower.pathBuilder()
                .addPath(new BezierLine(GrabLastChain, ScoreLastChain))
                .setConstantHeadingInterpolation(ScoreLastChain.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        GrabPlayer2 = follower.pathBuilder()
                .addPath(new BezierLine(ScoreLastChain, GrabPlayerZone2))
                .setTangentHeadingInterpolation()
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Scoreplayer2 = follower.pathBuilder()
                .addPath(new BezierLine(GrabPlayerZone2, ScorePlayerZone2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
            case 1:
                follower.followPath(grabPlayer);
                setPathState(2);
                break;
            case 2:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(ScorePlayer,true);
                    setPathState(2);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(GrabLast,true);
                    setPathState(3);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(ScoreLast,true);
                    setPathState(4);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(GrabPlayer2,true);
                    setPathState(5);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Scoreplayer2,true);
                    setPathState(6);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}