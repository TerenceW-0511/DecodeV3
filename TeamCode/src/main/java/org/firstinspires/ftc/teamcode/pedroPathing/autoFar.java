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
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Far Blue", group = "Blue")
public class autoFar extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Methods intakePID,transferPID,flywheelPID,methods;
    private Hardware robot;

    private boolean followerDoneDelay = false;

    private Timer followerDoneTimer = new Timer();
    private Timer shootingTimer = new Timer();



    private final Pose StartPose = new Pose(57.5, 6.7, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose GrabFirstChainPose = new Pose(24,34.5,Math.toRadians(120));
    private final Pose ScoreBallsPose = new Pose(50,11.2);
    private final Pose GrabPlayerPose = new Pose(11.2, 11.6, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose GrabBackPose = new Pose(23.97, 10.81, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose GrabPlayerPose2 = new Pose(9.09, 10.65, Math.toRadians(120)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose ScoreBallsPose2 = new Pose(49.1, 10.32, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose WaitForBallsPose = new Pose(11.46, 15.67, Math.toRadians(128)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    //    private final Pose regrabPlayer2 = new Pose(10,14,Math.toRadians(180));
//    private final Pose controlRetry2 = new Pose(25,10.9);

    private final Pose ControlToWaitForBallsPose = new Pose(21.15, 3.65, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose ScoreBallsPose3 = new Pose(49.18, 11.09, Math.toRadians(128)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose WaitForBallsPose2 = new Pose(11.32, 15.74, Math.toRadians(128)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose ScoreBallsPose4 = new Pose(48.5, 11.11, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose WaitForBallsPose3 = new Pose(11.74, 15.67, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose ScoreBallsPose5 = new Pose(49.06, 11.11, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    //    private Path scorePreload;
    private Path scorePreload;
    private PathChain GrabFirstChain,ScoreBalls,GrabPlayer,ScoreBalls2,WaitForballs,ScoreBalls3,WaitForballs2,ScoreBalls4,WaitForballs3,ScoreBalls5;

    public void buildPaths() {
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        GrabFirstChain = follower.pathBuilder()
                .addPath(new BezierLine(StartPose, GrabFirstChainPose))
                .setLinearHeadingInterpolation(StartPose.getHeading(), GrabFirstChainPose.getHeading())
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        ScoreBalls = follower.pathBuilder()
                .addPath(new BezierLine(GrabFirstChainPose, ScoreBallsPose))
                .setLinearHeadingInterpolation(GrabFirstChainPose.getHeading(), ScoreBallsPose.getHeading())
                .build();



        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        GrabPlayer = follower.pathBuilder()
                .addPath(new BezierLine(ScoreBallsPose,GrabPlayerPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(GrabBackPose,GrabPlayerPose2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        ScoreBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(GrabPlayerPose2, ScoreBallsPose2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        WaitForballs = follower.pathBuilder()
                .addPath(new BezierCurve(ScoreBallsPose2,ControlToWaitForBallsPose,WaitForBallsPose))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.5)
                .build();

        ScoreBalls3 = follower.pathBuilder()
                .addPath(new BezierLine(WaitForBallsPose,ScoreBallsPose3))
                .setLinearHeadingInterpolation(WaitForBallsPose.getHeading(), ScoreBallsPose3.getHeading())
                .build();

        WaitForballs2 = follower.pathBuilder()
                .addPath(new BezierCurve(ScoreBallsPose3,WaitForBallsPose2))
                .setLinearHeadingInterpolation(ScoreBallsPose3.getHeading(), WaitForBallsPose2.getHeading())
                .build();


//        Retry2 = follower.pathBuilder()
//                .addPath(new BezierCurve(GrabPlayerZone2,controlRetry2,regrabPlayer2))
//                .setConstantHeadingInterpolation(regrabPlayer2.getHeading())
//                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        ScoreBalls4 = follower.pathBuilder()
                .addPath(new BezierLine(WaitForBallsPose,ScoreBallsPose3))
                .setLinearHeadingInterpolation(WaitForBallsPose.getHeading(), ScoreBallsPose3.getHeading())
                .build();

        WaitForballs3 =  follower.pathBuilder()
                .addPath(new BezierCurve(ScoreBallsPose3,WaitForBallsPose2))
                .setLinearHeadingInterpolation(ScoreBallsPose3.getHeading(), WaitForBallsPose2.getHeading())
                .build();

        ScoreBalls5 =  follower.pathBuilder()
                .addPath(new BezierLine(WaitForBallsPose,ScoreBallsPose3))
                .setLinearHeadingInterpolation(WaitForBallsPose.getHeading(), ScoreBallsPose3.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (readyToOuttake(1))
                    if (outtake()){
                    setPathState(1);
                }
                break;
            case 1:
                follower.followPath(GrabFirstChain);
                setPathState(2);
                break;
            case 2:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(ScoreBalls);
                    setPathState(100);
                }
                break;
            case 100:
                if (readyToOuttake(1))
                    if (outtake()){
                        setPathState(4);
                }break;
            case 4:
                if (!follower.isBusy()){
                    follower.followPath(GrabPlayer);
                    setPathState(5);
                }break;
            case 5:
                if (follower.getPathCompletion()>0.3){
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(ScoreBalls2);
                    setPathState(99);
                }
                break;
            case 99:
                if (readyToOuttake(1)){
                    if (outtake()){
                        setPathState(6);
                    }
                }break;
            case 6:
                if (!follower.isBusy()){
                    follower.followPath(WaitForballs);
                    intake();
                    setPathState(7);
                }break;
            case 7:
                if (!follower.isBusy()){
                    follower.followPath(ScoreBalls3);
                    setPathState(98);
                }break;
            case 98:
                if (readyToOuttake(1)){
                    if (outtake()){
                        setPathState(8);
                    }
                }break;
            case 8:
                if (!follower.isBusy()){
                    follower.followPath(WaitForballs2);
                    intake();
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()){
                    follower.followPath(ScoreBalls4);
                    setPathState(101);
                }break;
            case 101:
                if (readyToOuttake(1)){
                    if (outtake()){
                        setPathState(10);
                    }
                }break;
            case 10:
                if (!follower.isBusy()){
                    follower.followPath(WaitForballs3);
                    intake();
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()){
                    follower.followPath(ScoreBalls5);
                    setPathState(12);
                }
        }
    }

    public void intake(){
//        follower.setMaxPower(0.4);
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        robot.intake.setPower(1);
        if (Values.frameCountBlockedTop>10){
            robot.transfer.setPower(0);
        }else {
            robot.transfer.setPower(1);
        }
    }

    //    public void gate(){
//        follower.setMaxPower(1);
//        robot.limiter.setPosition(Values.LIMITER_CLOSE);
//        robot.intake.setPower(1);
//        robot.transfer.setPower(.8);
//    }
    public void waitForBalls (double time){
        if (Values.topBlocked || pathTimer.getElapsedTimeSeconds() < time);
            
    }
    public void gate(double time){
        if (pathTimer.getElapsedTimeSeconds() < time) {
            robot.limiter.setPosition(Values.LIMITER_CLOSE);
            robot.intake.setPower(1);
            if (Values.frameCountBlockedTop>10){
                robot.transfer.setPower(0);
            }else {
                robot.transfer.setPower(1);
            }
        }
    }

    public void keepShoot(){
        if (Values.counter!=0) {
            robot.limiter.setPosition(Values.LIMITER_OPEN);
        }else{
            robot.limiter.setPosition(Values.LIMITER_CLOSE);
        }
        robot.intake.setPower(1);
        robot.transfer.setPower(1);

    }

    public void move(boolean intake){
//        if (follower.getPathCompletion()>0.7){
//            robot.limiter.setPosition(Values.LIMITER_OPEN);
//        }
        follower.setMaxPower(1);
//        if (follower.getPathCompletion()<0.2 && intake){
//            robot.limiter.setPosition(Values.LIMITER_CLOSE);
//            robot.intake.setPower(1);
//            robot.transfer.setPower(0.8);
//        }else{
//
//            robot.intake.setPower(0);
//            robot.transfer.setPower(0);
//        }

        if (intake){
            if (follower.getPathCompletion()>0.7 ){
                robot.limiter.setPosition(Values.LIMITER_OPEN);
            }else{
                robot.limiter.setPosition(Values.LIMITER_CLOSE);
            }
            if (follower.getPathCompletion()<.7){
                robot.intake.setPower(1);
                robot.transfer.setPower(0.8);
            }else{
                robot.intake.setPower(0);
                robot.transfer.setPower(0);
            }
        }else{
            robot.limiter.setPosition(Values.LIMITER_CLOSE);
            robot.intake.setPower(0);
            robot.transfer.setPower(0);

        }
    }
    public void setTurretPos(double pos){
        Values.turretPos = pos;
    }
    public boolean outtake(){
        ;
        double avgFlywheel = (robot.flywheel1.getVelocity() + robot.flywheel2.getVelocity()) / 2.0;
        double rpmError = Math.abs(avgFlywheel - Values.flywheel_Values.flywheelTarget);
        if (rpmError > 200){
            return false;
        }
        if (Values.counter!=0) {
            robot.intake.setPower(1);
            robot.transfer.setPower(1);
        }
        Values.counter=Values.oldcounter;
        return Values.counter==0;
    }
    public boolean readyToOuttake(double delay) {
        if (!follower.isBusy()) {
            if (!followerDoneDelay) {
                followerDoneDelay = true;
                followerDoneTimer.resetTimer();
                return false;
            }
            return followerDoneTimer.getElapsedTimeSeconds() > delay;
        }
        return false;
    }
    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
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
        double dist = methods.getDist(follower.getPose());
        autonomousPathUpdate();

        Values.autonFollowerX = follower.getPose().getX();
        Values.autonFollowerY = follower.getPose().getY();
        Values.autonHeading = follower.getHeading();

        Values.hoodPos = methods.hoodControl(follower,robot.flywheel1,robot.flywheel2);
        robot.hood1.setPosition(Values.hoodPos);

        double turretEncoder = robot.intake.getCurrentPosition();

        double targetTurret = methods.AutoAim(follower, robot.ll);
        Values.turretPos = methods.turretPID(turretEncoder, targetTurret + Values.turretOverride);
        robot.turret1.setPosition(Values.turretPos);
        robot.turret2.setPosition(Values.turretPos);

        double flywheelVel1 = robot.flywheel1.getVelocity();
        double flywheelVel2 = robot.flywheel2.getVelocity();
        Values.flywheel_Values.flywheelTarget = methods.flywheelControl(follower,robot.hood1.getPosition());
        flywheelPID.flywheelFFTele(robot.flywheel1, robot.flywheel2, Values.flywheel_Values.flywheelTarget);
        methods.countBalls(robot.breakBeam,robot.breakBeam2,robot.breakBeam3,robot.breakBeam4);
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);

        telemetry.addData("flywheel rpm", String.format("1: %f,2: %f",flywheelVel1,flywheelVel2));
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("count",Values.counter);
        telemetry.addData("states", methods.getstates(robot.breakBeam,robot.breakBeam2,robot.breakBeam3,robot.breakBeam4));
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
        shootingTimer = new Timer();


        robot = new Hardware(hardwareMap);
        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelPID = new Methods();
        methods = new Methods();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(StartPose);
        followerDoneTimer = new Timer();

        Values.team = Values.Team.BLUE;

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        double turretEncoder = robot.intake.getCurrentPosition();
        robot.hood1.setPosition(1);
//        Values.turretPos = methods.turretPID(turretEncoder, -8000);
        robot.turret1.setPosition(Values.turretPos);
        robot.turret2.setPosition(Values.turretPos);

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {

        opmodeTimer.resetTimer();
        robot.ll.pipelineSwitch(2);
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