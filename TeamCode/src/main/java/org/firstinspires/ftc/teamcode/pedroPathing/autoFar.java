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
    private double rpmError = 0;


    private final Pose startPose = new Pose(57.3, 6.5, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose shootPreloadPose = new Pose(50,8.6,Math.toRadians(180));
    private final Pose grabPose1 = new Pose(9.5,8.6,Math.toRadians(180));
    private final Pose scorePose1 = new Pose(56.7,16.6,Math.toRadians(190));
    private final Pose grabPose2 = new Pose(16.5,35.2,Math.toRadians(180));
    private final Pose controlPickup2 = new Pose(54.8,36);
    private final Pose scorePose2 = new Pose(50,8.6,Math.toRadians(142));

    //    private Path scorePreload;
    private Path grabPlayer;
    private PathChain shootPreload,grabBack,scoreBack,grabSecond,scoreSecond;

    public void buildPaths() {
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(startPose, GrabPlayerZone));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), GrabPlayerZone.getHeading());
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPreloadPose))
                .setConstantHeadingInterpolation(shootPreloadPose.getHeading())
                .build();
        grabBack = follower.pathBuilder()
                .addPath(new BezierLine(shootPreloadPose, grabPose1))
                .setTangentHeadingInterpolation()
                .build();
        scoreBack = follower.pathBuilder()
                .addPath(new BezierLine(grabPose1, scorePose1))
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




    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setTurretPos(-8000);
                follower.followPath(shootPreload);
                nextPath();
                break;
            case 1:
                robot.limiter.setPosition(Values.LIMITER_OPEN);
                if (!follower.isBusy() && rpmError<40){
                    if (outtake()){
                        nextPath();
                    }
                }
                break;
            case 2:
            case 8:
            case 11:
            case 14:
            case 17:
            case 20:
                if (!follower.isBusy()){
                    follower.followPath(grabBack);
                    nextPath();
                }break;
            case 3:
            case 9:
            case 12:
            case 15:
            case 18:
            case 21:
                if (follower.getPathCompletion()>0.5){
                    setTurretPos(-8000);
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(scoreBack);
                    nextPath();
                }
                break;
            case 4:
            case 7:
            case 10:
            case 13:
            case 16:
            case 19:
            case 22:
                move(true);
                if (!follower.isBusy()){
                    if (outtake()){
                        nextPath();
                    }
                }break;
            case 5:
                follower.followPath(grabSecond);
                nextPath();
                break;
            case 6:
                if (follower.getPathCompletion()>0.5){
                    setTurretPos(-8000);
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(scoreSecond);
                    nextPath();
                }
                break;


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
            if (follower.getPathCompletion()>0.2){
                robot.limiter.setPosition(Values.LIMITER_OPEN);
            }else{
                robot.limiter.setPosition(Values.LIMITER_CLOSE);
            }
            if (follower.getPathCompletion()<.1){
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
        targetTurret = pos;
    }
    public boolean outtake(){
        if (rpmError<40){
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

//        double targetTurret = methods.AutoAim(follower, robot.ll);
        Values.turretPos = methods.turretPID(turretEncoder, targetTurret + Values.turretOverride);
        robot.turret1.setPosition(Values.turretPos);
        robot.turret2.setPosition(Values.turretPos);

        double flywheelVel1 = robot.flywheel1.getVelocity();
        double flywheelVel2 = robot.flywheel2.getVelocity();
        double avgFlywheel = (flywheelVel1 + flywheelVel2) / 2.0;
        rpmError = Math.abs(avgFlywheel - Values.flywheel_Values.flywheelTarget);
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
        double turretEncoder = -robot.intake.getCurrentPosition();
        robot.hood1.setPosition(0);
        Values.turretPos = methods.turretPID(turretEncoder, -7000);
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