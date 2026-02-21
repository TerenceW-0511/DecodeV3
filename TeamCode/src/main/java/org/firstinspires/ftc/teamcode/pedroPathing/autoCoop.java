package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

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

@Autonomous(name = "Close Coop", group = "Blue")
public class autoCoop extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Hardware robot;
    private Methods intakePID,transferPID,flywheelPID,methods;

    private int pathState;
    private final Pose startPose = new Pose(15.5, 113.5, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(49, 115, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1Pose = new Pose(17.3, 84.3, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup1 = new Pose(49.5,87.8);
    private final Pose gatePose = new Pose(14,71,Math.toRadians(180));
    private final Pose gateControl1 = new Pose(29.1,76);


    private final Pose scorePickup1Pose = new Pose(50,84.3,Math.toRadians(180));
    private final Pose controlScore1 = new Pose(46.5,60.7);

    private final Pose pickup2Pose = new Pose(6, 59.1, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup2 = new Pose(46.5,60.7);
    private final Pose gateControl2 = new Pose(26.5,63.2);

    private final Pose scorePickup2Pose = new Pose(50.2,85,Math.toRadians(180));
    private final Pose controlScore2 = new Pose(46.5,73);


    private final Pose pickup3Pose = new Pose(6, 50, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup3 = new Pose(45.7,50.5);
    private final Pose gateControl3 = new Pose(33.1,68.7);
    private final Pose scorePickup3Pose = new Pose(55,85,Math.toRadians(180));

//    private final Pose pickup4Pose = new Pose(9, 34.1, Math.toRadians(180));
//    private final Pose control41 = new Pose(7.4,65.3);
//
    private final Pose scorePickup4Pose = new Pose(51.8,110.3,Math.toRadians(180));
    private final Pose leavePose = new Pose(25,70.9,Math.toRadians(180));
    private final Pose controlLeave = new Pose(37,71.3);

    private Path scorePreload;

    private PathChain grabPickup1, scorePickup1, gate1,grabPickup2, gate2,scorePickup2, grabPickup3, gate3,scorePickup3,grabPickup4,scorePickup4;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(scorePose.getHeading());

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,controlPickup1, pickup1Pose))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        gate1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose,gateControl1,gatePose))
                .setConstantHeadingInterpolation(gatePose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, scorePickup1Pose))
                .setConstantHeadingInterpolation(scorePickup1Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePickup1Pose,controlPickup2, pickup2Pose))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        gate2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose,gateControl2, gatePose))
                .setConstantHeadingInterpolation(gatePose.getHeading())
                .build();


        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, controlPickup2,scorePickup2Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePickup2Pose, controlPickup3,pickup3Pose))
                .setTangentHeadingInterpolation()
                .build();

        gate3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose,gateControl3, gatePose))
                .setConstantHeadingInterpolation(gatePose.getHeading())
                .build();


        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose,scorePickup4Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

//        grabPickup4 = follower.pathBuilder()
//                .addPath(new BezierCurve(scorePickup3Pose,control41,pickup4Pose))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        scorePickup4 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup4Pose,scorePickup4Pose))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();




    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                move(true);
                if (!follower.isBusy()){
                    if (outtake(2,2)){
                        setPathState(2);
                    }
                }break;
            case 2:
                move(false);
                if (!follower.isBusy()){
                    follower.followPath(grabPickup1);
                    setPathState(3);
                }break;
            case 3:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(gate1);
                    setPathState(4);
                }break;
            case 4:
                gate();
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2){
                    follower.followPath(scorePickup1);
                    setPathState(5);
                }break;
            case 5:
                move(false);
                if (!follower.isBusy()){
                    if (outtake(2,2)){
                        setPathState(6);
                    }
                }
                break;
            case 6:
                move(false);
                if (!follower.isBusy()){
                    follower.followPath(grabPickup2);
                    setPathState(7);
                }break;
            case 7:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(gate2);
                    setPathState(8);
                }break;
            case 8:
            case 12:
                gate();
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2){
                    follower.followPath(scorePickup2);
                    nextPath();
                }break;
            case 9:
            case 13:
                move(false);
                if (!follower.isBusy()){
                    if (outtake(2,2.5)){
                        nextPath();
                    }
                }break;
            case 10:
            case 14:
                move(false);
                if (!follower.isBusy()){
                    follower.followPath(grabPickup3);
                    nextPath();
                }break;
            case 11:
                if (follower.getPathCompletion()>0.7){
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(gate3);
                    setPathState(12);
                }break;
            case 15:
                if (follower.getPathCompletion()>0.7){
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(scorePickup3);
                    setPathState(16);
                }
            case 16:
                move(true);
                if (!follower.isBusy()){
                    if (outtake(2,2)){
                        setPathState(17);
                    }
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
        if (pathTimer.getElapsedTimeSeconds()>wait) {
            robot.intake.setPower(1);
            robot.transfer.setPower(1);
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
        Values.team = Values.Team.BLUE;

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        double turretEncoder = -robot.intake.getCurrentPosition();

        Values.turretPos = methods.turretPID(turretEncoder, -8000);
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