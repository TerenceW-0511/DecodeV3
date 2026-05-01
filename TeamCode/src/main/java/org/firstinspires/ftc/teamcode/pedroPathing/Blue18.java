package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

@Autonomous(name = "B18 Consistent", group = "Blue")
public class Blue18 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, followerDoneTimer;
    private Hardware robot;


    private Methods intakePID,transferPID,flywheelPID,methods;
    private boolean followerDoneDelay = false;
    private double curr=0,rpmError = 0;
    private Timer shootingTimer = new Timer();
    private int pathState;
    private double targetTurret;


    private final Pose startPose = new Pose(15.8,114.2,Math.toRadians(225));
    private final Pose scorepreload = new Pose(44.5, 114.2, Math.toRadians(270));
    private final Pose toPickup1 = new Pose(43.2,60);
    private final Pose pickup1Pose = new Pose(12, 60, Math.toRadians(180));

    private final Pose tapGatePose = new Pose(16,55,Math.toRadians(160));

    private final Pose ControlTapPose = new Pose(26,69.2);

    private final Pose score1Pose = new Pose(52.3,79.3,Math.toRadians(180));

    private final Pose gatePose = new Pose(17.2,65.5,Math.toRadians(180));

    private final Pose controlGate1 = new Pose(39.6,70.8);
    private final Pose gateBackPose = new Pose(13.1,61.2,Math.toRadians(160));

    private final Pose scoreGatePose = new Pose(57.8,78.3,Math.toRadians(180));

    private final Pose pickup2Pose = new Pose(20.7,84.3,Math.toRadians(180));

    private final Pose score2Pose = new Pose(46,84.3,Math.toRadians(180));


//    private final Pose toLoadingPose = new Pose(7,35.5,Math.toRadians(225));
//    private final Pose pickup4Pose = new Pose(9,50,Math.toRadians(270));
//    private final Pose controlPickup4 = new Pose(36.4,50.5);
//
    ////    private final Pose scorePickup4Pose = new Pose(51,117.6,Math.toRadians(180));
//    private final Pose controlScore4Pose = new Pose(38.8,76.6);
    private PathChain scorePreload,grabPickup1,tapGate,scorePickup1,gate1,scoreGates,gate2,grabPickup2,scorePickup2,gate3;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose,scorepreload))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorepreload,toPickup1))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(
                                   0,
                                   0.8,
                                   HeadingInterpolator.tangent
                            ),
                            new HeadingInterpolator.PiecewiseNode(
                                    0.8,
                                    1,
                                    HeadingInterpolator.linear(scorepreload.getHeading(),pickup1Pose.getHeading())
                            )
                        )
                        )
                .addPath(new BezierLine(toPickup1,pickup1Pose))
                .setTangentHeadingInterpolation()
                .build();
        tapGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose,ControlTapPose,tapGatePose))
                .setConstantHeadingInterpolation(tapGatePose.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose,score1Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        gate1 = follower.pathBuilder()
                .addPath(new BezierCurve(score1Pose,controlGate1,gatePose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(gatePose,gateBackPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateBackPose.getHeading())
                .build();

        scoreGates = follower.pathBuilder()
                .addPath(new BezierLine(gateBackPose,gatePose))
                .setLinearHeadingInterpolation(gateBackPose.getHeading(),gatePose.getHeading())
                .addPath(new BezierLine(gatePose,scoreGatePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        gate2 = follower.pathBuilder()
                .addPath(new BezierLine(scoreGatePose,gatePose))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1,
                                        HeadingInterpolator.linear(scoreGatePose.getHeading(),gatePose.getHeading())
                                )
                        )

                )
                .addPath(new BezierLine(gatePose,gateBackPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateBackPose.getHeading())
                .build();



        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scoreGatePose,pickup2Pose))
                .setTangentHeadingInterpolation()
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose,score2Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        gate3 = follower.pathBuilder()
                .addPath(new BezierLine(score2Pose,gatePose))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        0.9,
                                        HeadingInterpolator.linear(score2Pose.getHeading(),gatePose.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.9,
                                        1,
                                        HeadingInterpolator.tangent
                                )
                        )
                )
                .addPath(new BezierLine(gatePose,gateBackPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateBackPose.getHeading())
                .build();






    }
    private void autonomousPathUpdate(){

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
        if (Values.counter!=0) {
            robot.intake.setPower(1);
            robot.transfer.setPower(1);
        }
        if (Values.counter<Values.oldcounter && Values.counter==1){
            return true;
        }
        Values.oldcounter=Values.counter;
        return Values.counter==0;
    }

    public boolean instantOuttake(){
        if (Values.counter!=0) {
            robot.intake.setPower(1);
            robot.transfer.setPower(1);
        }
        return true;
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
        followerDoneDelay=false;
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

//        double targetTurret = methods.AutoAim(follower, robot.ll);
        Values.turretPos = methods.turretPID(turretEncoder, targetTurret + Values.turretOverride);
        robot.turret1.setPosition(Values.turretPos);
        robot.turret2.setPosition(Values.turretPos);

        double flywheelVel1 = robot.flywheel1.getVelocity();
        double flywheelVel2 = robot.flywheel2.getVelocity();

        Values.flywheel_Values.flywheelTarget = methods.flywheelControl(follower,robot.hood1.getPosition());

        curr=flywheelPID.flywheelFFTele(robot.flywheel1, robot.flywheel2, Values.flywheel_Values.flywheelTarget);
        rpmError = Math.abs(curr - Values.flywheel_Values.flywheelTarget);
        methods.countBalls(robot.breakBeam,robot.breakBeam2,robot.breakBeam3,robot.breakBeam4,true);
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("turret",targetTurret);
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
        followerDoneTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        shootingTimer = new Timer();
        Values.farCoded=false;


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
        double turretEncoder = robot.intake.getCurrentPosition();
        robot.hood1.setPosition(1);
        Values.turretPos = methods.turretPID(turretEncoder, -12400);
        robot.turret1.setPosition(Values.turretPos);
        robot.turret2.setPosition(Values.turretPos);
        telemetry.addData("turret",turretEncoder);
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
//        robot.ll.pipelineSwitch(2);
//        robot.ll.start();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
//        robot.ll.stop();
    }
}