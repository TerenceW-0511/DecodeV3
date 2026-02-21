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

@Autonomous(name = "Far Auto Red", group = "Red")
public class autoFarRed extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Methods intakePID,transferPID,flywheelPID,methods;
    private Hardware robot;


    private final Pose startPose = mirrorPose(new Pose(57.5, 6.7, Math.toRadians(180))); // Start Pose of our robot.
    private final Pose GrabPlayerZone = mirrorPose( new Pose(10.325581395348829, 8.46511627906977, Math.toRadians(180))); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose regrabPlayer = mirrorPose(new Pose(10.4,12,Math.toRadians(180)));
    private final Pose controlRetry = mirrorPose(new Pose(24.6,9.3));
    private final Pose ScorePlayerZone = mirrorPose(new Pose(58.527906976744184, 15.406976744186043, Math.toRadians(180))); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose GrabLastChain = mirrorPose(new Pose(17.069767441860442, 37.06976744186046, Math.toRadians(180))); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose ControlToGrabLastChain = mirrorPoint(new Pose(55.06976744186045, 37.77906976744185)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose ScoreLastChain = mirrorPose(new Pose(49.4, 8.5, Math.toRadians(180))); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose GrabPlayerZone2 = mirrorPose( new Pose(10.3, 8.5, Math.toRadians(180))); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose regrabPlayer2 = mirrorPose(new Pose(10,14,Math.toRadians(180)));
    private final Pose controlRetry2 = mirrorPoint(new Pose(25,10.9));
    private final Pose ScorePlayerZone2 = mirrorPose(new Pose(49.4, 8.5, Math.toRadians(180))); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose leave = mirrorPose(new Pose(37.1,8.5));

    //    private Path scorePreload;
    private Path grabPlayer;
    private PathChain Retry,ScorePlayer, GrabLast, ScoreLast, GrabPlayer2,Retry2, Scoreplayer2,Leave;

    public void buildPaths() {
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(startPose, GrabPlayerZone));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), GrabPlayerZone.getHeading());
        grabPlayer = new Path(new BezierLine(startPose, GrabPlayerZone));
        grabPlayer.setLinearHeadingInterpolation(startPose.getHeading(), GrabPlayerZone.getHeading());
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        Retry = follower.pathBuilder()
                .addPath(new BezierCurve(GrabPlayerZone,controlRetry,regrabPlayer))
                .setConstantHeadingInterpolation(regrabPlayer.getHeading())
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        ScorePlayer = follower.pathBuilder()
                .addPath(new BezierLine(regrabPlayer, ScorePlayerZone))
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

        Retry2 = follower.pathBuilder()
                .addPath(new BezierCurve(GrabPlayerZone2,controlRetry2,regrabPlayer2))
                .setConstantHeadingInterpolation(regrabPlayer2.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Scoreplayer2 = follower.pathBuilder()
                .addPath(new BezierLine(GrabPlayerZone2, ScorePlayerZone2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Leave =  follower.pathBuilder()
                .addPath(new BezierLine(ScorePlayerZone2,leave))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (outtake(1,4)){
                    setPathState(1);
                }
                break;
            case 1:
                follower.followPath(grabPlayer);
                setPathState(2);
                break;
            case 2:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(Retry);
                    setPathState(3);
                }
                break;
            case 3:
                intake();
                if (!follower.isBusy()) {
                    follower.followPath(ScorePlayer);
                    setPathState(4);
                }break;
            case 4:
                move(true);
                if (!follower.isBusy()) {
                    if (outtake(2, 4)) {
                        setPathState(5);
                    }
                }break;
            case 5:
                move(false);
                if (!follower.isBusy()){
                    follower.followPath(GrabLast);
                    setPathState(6);
                }break;
            case 6:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(ScoreLast);
                    setPathState(7);
                }break;
            case 7:
                move(true);
                if (!follower.isBusy()){
                    if (outtake(2,4)){
                        setPathState(8);
                    }
                }break;
            case 8:
            case 12:
                move(false);
                if (!follower.isBusy()){
                    follower.followPath(GrabPlayer2);
                    nextPath();
                }
                break;
            case 9:
            case 13:
                if (follower.getPathCompletion()>0.5) {
                    intake();
                }
                if (!follower.isBusy()) {
                    follower.followPath(Retry2);
                    nextPath();
                }break;
            case 10:
            case 14:
                intake();
                if (!follower.isBusy()) {
                    follower.followPath(Scoreplayer2);
                    nextPath();
                }
            case 11:
            case 15:
                move(true);
                if (!follower.isBusy()){
                    if (outtake(2,4)){
                        nextPath();
                    }
                }break;
            case 16:
                move(false);
                if (!follower.isBusy()){
                    follower.followPath(Leave);
                    setPathState(17);
                }





        }
    }
    public void intake(){
        follower.setMaxPower(0.4);
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        robot.intake.setPower(1);
        robot.transfer.setPower(.8);

    }

    public void gate(){
        follower.setMaxPower(1);
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        robot.intake.setPower(1);
        robot.transfer.setPower(.8);
    }

    public void move(boolean intake){
        if (follower.getPathCompletion()>0.7){
            robot.limiter.setPosition(Values.LIMITER_OPEN);
        }
        follower.setMaxPower(1);
        if (follower.getPathCompletion()<0.2 && intake){
            robot.limiter.setPosition(Values.LIMITER_CLOSE);
            robot.intake.setPower(1);
            robot.transfer.setPower(0.8);
        }else{
            robot.intake.setPower(0);
            robot.transfer.setPower(0);
        }
    }
    public boolean outtake(double wait,double time){
        ;
        double avgFlywheel = (robot.flywheel1.getVelocity() + robot.flywheel2.getVelocity()) / 2.0;
        double rpmError = Math.abs(avgFlywheel - Values.flywheel_Values.flywheelTarget);
        if (pathTimer.getElapsedTimeSeconds()>wait && rpmError<70) {
            robot.intake.setPower(1);
            robot.transfer.setPower(1);
        }else{
            robot.intake.setPower(0);
            robot.transfer.setPower(0);
        }
        return pathTimer.getElapsedTimeSeconds() > time+1;
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void nextPath(){
        pathState += 1;
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
        follower.setStartingPose(startPose);
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
        robot.ll.start();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}