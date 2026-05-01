package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.security.UnrecoverableEntryException;

@Autonomous(name = "Far Blue", group = "Blue")
public class autoFar extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Methods intakePID,transferPID,flywheelPID,methods;
    private Hardware robot;
    private double targetTurret;
    private double rpmError = 0,curr=0;


    private final Pose startPose = new Pose(40, 6.5, Math.toRadians(180)); // Start Pose of our robot.
    //    private final Pose shootPreloadPose = new Pose(40,8.6,Math.toRadians(180));
    private final Pose grabPose1 = new Pose(9.5,6.5,Math.toRadians(180));
    private final Pose regrabPose = new Pose(9.5,10,Math.toRadians(180));
    private final Pose controlRegrab = new Pose(23.9,8.2);
    private final Pose scorePose1 = new Pose(56.7,16.6,Math.toRadians(190));
    private final Pose grabPose2 = new Pose(16.5,35.2,Math.toRadians(180));
    private final Pose grabPose3 = new Pose(7,32.5,Math.toRadians(97));
    private final Pose controlPickup3 = new Pose(10.3,4);
    private final Pose controlPickup2 = new Pose(54.8,36);
    private final Pose scorePose2 = new Pose(50,8.6,Math.toRadians(142));
    private final Pose grabPoseMiddler = new Pose(10,28.9,Math.toRadians(109));
    private final Pose controlMiddler = new Pose(17.3,8.6);


    //    private Path scorePreload;
    private Path grabPlayer;
    private PathChain shootPreload,grabBack,regrab,scoreBack,grabSecond,scoreSecond,grabFurther,grabMiddler,scoreMiddler;

    public void buildPaths() {
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(startPose, GrabPlayerZone));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), GrabPlayerZone.getHeading());
//        shootPreload = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, grabPose1))
//                .setConstantHeadingInterpolation(grabPose1.getHeading())
//                .setTranslationalConstraint(1)
//                .build();
        grabBack = follower.pathBuilder()
                .addPath(new BezierLine(startPose, grabPose1))
                .setTangentHeadingInterpolation()
                .build();
        regrab = follower.pathBuilder()
                .addPath(new BezierCurve(grabPose1,controlRegrab,regrabPose))
                .setConstantHeadingInterpolation(grabPose1.getHeading())
                .build();
        scoreBack = follower.pathBuilder()
                .addPath(new BezierLine(regrabPose, scorePose1))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        grabSecond = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose1, controlPickup2,grabPose2))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.5,
                                        HeadingInterpolator.linear(scorePose1.getHeading(),grabPose2.getHeading())),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.5,
                                        1,
                                        HeadingInterpolator.tangent
                                )
                        )
                )
                .build();
        scoreSecond = follower.pathBuilder()
                .addPath(new BezierLine(grabPose2, scorePose2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        grabFurther = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2,grabPose1))
                .setTangentHeadingInterpolation()
                .build();
        grabMiddler = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose2,controlMiddler,grabPoseMiddler))
                .setTangentHeadingInterpolation()
                .build();
        scoreMiddler = follower.pathBuilder()
                .addPath(new BezierLine(grabPoseMiddler,scorePose2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

//



    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                break;
            case 1:
                robot.limiter.setPosition(Values.LIMITER_OPEN);
                if (!follower.isBusy() && rpmError<20){
                    if (outtake()){
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    follower.followPath(grabBack);
                    setPathState(3);
                }break;

            case 3:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }else{
                    move(false);
                }
                if (!follower.isBusy()){
                    follower.followPath(regrab);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    follower.followPath(scoreBack);
                    setPathState(5);
                }
                break;

            case 5:
                move(true);
                if (!follower.isBusy()){
                    setPathState(100);
                }
                break;
            case 100:
                if (pathTimer.getElapsedTimeSeconds()>0.2){
                    if (outtake()){
                        setPathState(6);
                    }
                }break;
            case 6:
                follower.followPath(grabSecond);
                setPathState(7);
                break;
            case 7:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }else{
                    move(false);
                }
                if (!follower.isBusy()){
                    follower.followPath(scoreSecond);
                    setPathState(8);
                }
                break;
            case 8:
                move(true);
                if (!follower.isBusy()){
                    setPathState(101);
                }
                break;
            case 101:
                if (pathTimer.getElapsedTimeSeconds()>0.2){
                    if (outtake()){
                        setPathState(9);
                    }
                }break;
            case 9:
                follower.followPath(grabFurther);
                setPathState(10);
                break;
            case 10:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }else{
                    move(false);
                }
                if (!follower.isBusy()){
                    follower.followPath(regrab);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()){
                    follower.followPath(scoreBack);
                    setPathState(12);
                }break;
            case 12:
                move(true);
                if (!follower.isBusy()){
                    setPathState(102);
                }break;
            case 102:
                if (pathTimer.getElapsedTimeSeconds()>0.2){
                    if (outtake()){
                        setPathState(13);
                    }
                }break;
            case 13:
                follower.followPath(grabMiddler);
                setPathState(14);
                break;
            case 14:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }else{
                    move(false);
                }
                if (!follower.isBusy()){
                    follower.followPath(scoreMiddler);
                    setPathState(15);
                }break;
            case 15:
                move(true);
                if (!follower.isBusy()){
                    setPathState(103);
                }break;
            case 103:
                if (pathTimer.getElapsedTimeSeconds()>0.2){
                    if (outtake()){
                        setPathState(16);
                    }
                }break;
            case 16:
                follower.followPath(grabFurther);
                setPathState(17);
                break;
            case 17:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }else{
                    move(false);
                }
                if (!follower.isBusy()){
                    follower.followPath(regrab);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()){
                    follower.followPath(scoreBack);
                    setPathState(19);
                }break;
            case 19:
                move(true);
                if (!follower.isBusy()){
                    setPathState(104);
                }break;
            case 104:
                if (pathTimer.getElapsedTimeSeconds()>0.2){
                    if (outtake()){
                        setPathState(20);
                    }
                }break;




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
        robot.limiter.setPosition(Values.LIMITER_OPEN);
//        if (Values.counter!=0) {
//            robot.limiter.setPosition(Values.LIMITER_OPEN);
//        }else{
//            robot.limiter.setPosition(Values.LIMITER_CLOSE);
//        }
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
            if (follower.getPathCompletion()>0.8){
                robot.limiter.setPosition(Values.LIMITER_OPEN);
            }else{
                robot.limiter.setPosition(Values.LIMITER_CLOSE);
            }
            if (follower.getPathCompletion()<.6){
                robot.intake.setPower(1);
                robot.transfer.setPower(0.8);
            }else{
                robot.intake.setPower(0);
                robot.transfer.setPower(0);
            }
        }else{
            robot.limiter.setPosition(Values.LIMITER_CLOSE);
            if (follower.getPathCompletion()>0.2) {
                robot.intake.setPower(1);
                robot.transfer.setPower(1);
            }else {
                robot.intake.setPower(0);
                robot.transfer.setPower(0);
            }

        }
    }
    public boolean outtake(){
        if (rpmError<20){
            robot.intake.setPower(1);
            robot.transfer.setPower(1);
        }else{
            robot.intake.setPower(0);
            robot.transfer.setPower(0);
        }
        return Values.counter==0;
    }

    public boolean instantOuttake(){
        if (Values.counter!=0) {
            robot.intake.setPower(1);
            robot.transfer.setPower(1);
        }
        return true;
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void   nextPath(){
        pathState += 1;
        pathTimer.resetTimer();
    }

    public void skipPath(){
        pathState+=2;
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

        Values.hoodPos = 0.24;
        robot.hood1.setPosition(Values.hoodPos);

        double turretEncoder = robot.intake.getCurrentPosition();

        double targetTurret = methods.AutoAim(follower, robot.ll);
        Values.turretPos = methods.turretPID(turretEncoder, targetTurret + Values.turretOverride);
        robot.turret1.setPosition(Values.turretPos);
        robot.turret2.setPosition(Values.turretPos);

        double flywheelVel1 = robot.flywheel1.getVelocity();
        double flywheelVel2 = robot.flywheel2.getVelocity();

        Values.flywheel_Values.flywheelTarget = methods.flywheelControl(follower,Values.hoodPos);
        curr=flywheelPID.flywheelFFTele(robot.flywheel1, robot.flywheel2, Values.flywheel_Values.flywheelTarget);
        rpmError = Math.abs(curr - Values.flywheel_Values.flywheelTarget);
        methods.countBalls(robot.breakBeam,robot.breakBeam2,robot.breakBeam3,robot.breakBeam4,false);
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("target",Values.flywheel_Values.flywheelTarget);
        telemetry.addData("curr",curr);
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
        Values.farCoded=true;

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
        robot.limiter.setPosition(Values.LIMITER_OPEN);
        double turretEncoder = robot.intake.getCurrentPosition();
        robot.hood1.setPosition(0);
        Values.turretPos = methods.turretPID(turretEncoder, -7200);
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