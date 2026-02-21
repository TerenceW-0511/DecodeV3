package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Example Auto", group = "Blue")
public class autoFarRed extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Methods intakePID,transferPID,flywheelPID,methods;
    private Hardware robot;


    private final Pose startPose = new Pose(57.88372093023256, 8.46511627906977, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose GrabPlayerZone = new Pose(10.325581395348829, 8.46511627906977, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose ScorePlayerZone = new Pose(58.527906976744184, 15.406976744186043, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose GrabLastChain = new Pose(17.069767441860442, 37.06976744186046, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose ControlToGrabLastChain = new Pose(55.06976744186045, 37.77906976744185); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose ScoreLastChain = new Pose(49.3953488372093, 8.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose GrabPlayerZone2 = new Pose(10.3, 8.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose ScorePlayerZone2 = new Pose(49.4, 8.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose startPoseRed = mirrorPose(startPose);// Start Pose of our robot.
    private final Pose GrabPlayerZoneRed = mirrorPose(GrabPlayerZone); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose ScorePlayerZoneRed = mirrorPose(ScorePlayerZone); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose GrabLastChainRed = mirrorPose(GrabLastChain); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose ControlToGrabLastChainRed = mirrorPoint(ControlToGrabLastChain); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose ScoreLastChainRed = mirrorPose(ScoreLastChain); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose GrabPlayerZone2Red = mirrorPose(GrabPlayerZone2); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose ScorePlayerZone2Red = mirrorPose(ScorePlayerZone2); // Lowest (Third Set) of Artifacts from the Spike Mark.
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
                .addPath(new BezierLine(GrabPlayerZoneRed, ScorePlayerZoneRed))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        GrabLast = follower.pathBuilder()
                .addPath(new BezierCurve(ScorePlayerZoneRed, ControlToGrabLastChainRed,GrabLastChainRed))
                .setTangentHeadingInterpolation()
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        ScoreLast = follower.pathBuilder()
                .addPath(new BezierLine(GrabLastChainRed, ScoreLastChainRed))
                .setConstantHeadingInterpolation(ScoreLastChainRed.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        GrabPlayer2 = follower.pathBuilder()
                .addPath(new BezierLine(ScoreLastChainRed, GrabPlayerZone2Red))
                .setTangentHeadingInterpolation()
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Scoreplayer2 = follower.pathBuilder()
                .addPath(new BezierLine(GrabPlayerZone2Red, ScorePlayerZone2Red))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (robot.flywheel1.getVelocity() + robot.flywheel2.getVelocity() /2 >= Values.flywheel_Values.flywheelTarget && outtake(3)) {
                        setPathState(1);
                }
                break;
            case 1:
                move();
                follower.followPath(grabPlayer);
                setPathState(69);
                break;
            case 69:
                if (follower.getPathCompletion()>5) {
                    intake();
                    setPathState(2);
                }
                break;
            case 2:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                if(!follower.isBusy()) {
                    follower.followPath(ScorePlayer);
                    setPathState(37);
                }
                break;
            case 37:
                if (outtake(3)){
                    setPathState(3);
                }
            case 3:
                move();
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    follower.followPath(GrabLast);
                    setPathState(44);
                }
                break;
            case 44:
                if (follower.getPathCompletion()>5){
                    intake();
                    setPathState(4);
                }
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    follower.followPath(ScoreLast);
                    setPathState(58);
                }
            case 58:
                if (outtake(3)){
                    setPathState(5);
                }
                break;
            case 9:
            case 5:
                move();
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(GrabPlayer2);
                    nextPath();
                }
                break;
            case 10:
            case 6:
                if (follower.getPathCompletion()>.5){
                    intake();
                    nextPath();
                }
            case 11:
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    follower.followPath(Scoreplayer2);
                    nextPath();
                }
                break;
            case 12:
            case 8:
                if (outtake(3)){
                    nextPath();}
                break;
            case 13:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }
    public void intake(){
        follower.setMaxPower(0.4);
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        robot.intake.setPower(1);
        robot.transfer.setPower(.8);

    }

    public void move(){
        robot.limiter.setPosition(Values.LIMITER_OPEN);
        follower.setMaxPower(1);
        robot.intake.setPower(0);
        robot.transfer.setPower(0);
    }
    public boolean outtake(double time){
        double avgFlywheel = (robot.flywheel1.getVelocity() + robot.flywheel2.getVelocity()) / 2.0;
        double rpmError = Math.abs(avgFlywheel - Values.flywheel_Values.flywheelTarget);
        if (pathTimer.getElapsedTimeSeconds()>0.2) {
            robot.intake.setPower(1);
            robot.transfer.setPower(1);
        }
        if (pathTimer.getElapsedTimeSeconds()>time){
            return true;
        }
        return false;
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void nextPath(){
        pathTimer.resetTimer();
        pathState+=1;
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        Values.autonFollowerX = follower.getPose().getX();
        Values.autonFollowerY = follower.getPose().getY();
        Values.autonHeading = follower.getHeading();

        Values.hoodPos = methods.hoodControl(follower,robot.flywheel1,robot.flywheel2);
        robot.hood1.setPosition(Values.hoodPos);

        double turretEncoder = -robot.intake.getCurrentPosition();

        double targetTurret = methods.AutoAim(follower.getPose(), robot.ll);
        Values.turretPos = methods.turretPID(turretEncoder, targetTurret + Values.turretOverride);
        robot.turret1.setPosition(Values.turretPos);
        robot.turret2.setPosition(Values.turretPos);

        Values.flywheel_Values.flywheelTarget = methods.flywheelControl(follower,robot.hood1.getPosition());
        flywheelPID.flywheelFFTele(robot.flywheel1, robot.flywheel2, Values.flywheel_Values.flywheelTarget);

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

        robot = new Hardware(hardwareMap);
        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelPID = new Methods();
        methods = new Methods();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPoseRed);
        Values.team = Values.Team.RED;

    }
    public static Pose mirrorPose(Pose p) {
        return new Pose(
                144 - p.getX(),
                p.getY(),
                Math.PI - p.getHeading()
        );
    }

    public static Pose mirrorPoint(Pose p) {
        return new Pose(
                144 - p.getX(),
                p.getY()
        );
    }
    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        double turretEncoder = -robot.intake.getCurrentPosition();

        Values.turretPos = methods.turretPID(turretEncoder, -4000);
        robot.turret1.setPosition(Values.turretPos);
        robot.turret2.setPosition(Values.turretPos);
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