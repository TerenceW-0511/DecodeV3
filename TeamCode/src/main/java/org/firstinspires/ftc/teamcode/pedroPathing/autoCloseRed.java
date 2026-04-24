package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

@Autonomous(name = "Close 15 Blue", group = "Blue")
public class autoCloseRed extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, followerDoneTimer;
    private Hardware robot;


    private Methods intakePID,transferPID,flywheelPID,methods;
    private boolean followerDoneDelay = false;
    private double rpmError = 0;
    private Timer shootingTimer = new Timer();
    private int pathState;
    private double targetTurret;
    private final Pose startPose = new Pose(31, 134.5, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(49.5, 102.5, Math.toRadians(270));
    private final Pose pickup1Pose = new Pose(16, 60.7, Math.toRadians(180)); // -140 Highest (First  Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup1 = new Pose(45.7,61.1);
//    private final Pose controlPickup1_2 = new Po\e(39.6,69.6);

    private final Pose scorePickup1Pose = new Pose(54.6,80.9,Math.toRadians(210));

    private final Pose openGatePose = new Pose(11,64,Math.toRadians(160));
//    private final Pose controlGate = new Pose(29.2,65.2);

    private final Pose scorePickup2Pose = new Pose(51.0,79.7,Math.toRadians(210));

    private final Pose openGatePose2 = new Pose(11,64,Math.toRadians(155));

    private final Pose scorePickup3Pose = new Pose(57.1,83.4,Math.toRadians(210));

    private final Pose pickup2Pose = new Pose(18, 83.4, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    //    private final Pose controlPickup2 = new Pose(46.5,60.7);
    private final Pose scorePickup4Pose = new Pose(57.2,77.4,Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(13, 37.3, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup3 = new Pose(54.5,  31.8);
//    private final Pose controlPickup3_2 = new Pose(36,38);

    private final Pose scorePickup5Pose = new Pose(53.4,112.7,Math.toRadians(180));


//    private final Pose toLoadingPose = new Pose(7,35.5,Math.toRadians(225));
//    private final Pose pickup4Pose = new Pose(9,50,Math.toRadians(270));
//    private final Pose controlPickup4 = new Pose(36.4,50.5);
//
    ////    private final Pose scorePickup4Pose = new Pose(51,117.6,Math.toRadians(180));
//    private final Pose controlScore4Pose = new Pose(38.8,76.6);
    private Path scorePreload;

    private PathChain grabPickup1, scorePickup1, openGate,scorePickup2, openGate2,scorePickup3,grabPickup2,scorePickup4,grabPickup3,ScorePickup5;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(scorePose.getHeading());

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPickup1,pickup1Pose))
                .setTangentHeadingInterpolation()

                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose,scorePickup1Pose))
                .setTranslationalConstraint(2)
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(scorePickup1Pose, openGatePose))
                .setLinearHeadingInterpolation(scorePickup1Pose.getHeading(),  openGatePose.getHeading())
                .setTranslationalConstraint(0.1)
                .setHeadingConstraint(0.1)
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, scorePickup2Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        openGate2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePickup2Pose, openGatePose2))
                .setLinearHeadingInterpolation(scorePickup2Pose.getHeading(),  openGatePose.getHeading())
                .setTranslationalConstraint(0.1)
                .setHeadingConstraint(0.1)
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, scorePickup3Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();


        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePickup3Pose, pickup2Pose))
                .setTangentHeadingInterpolation()
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePickup4Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePickup4Pose,controlPickup3,pickup3Pose))
                .setConstantHeadingInterpolation(pickup3Pose.getHeading())
                .build();

        ScorePickup5 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose,scorePickup5Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();





    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setTurretPos(-12600);
                follower.followPath(scorePreload);
                setPathState(98);
                break;
            case 98:
                robot.limiter.setPosition(Values.LIMITER_OPEN);
//                move(false);
                if (!follower.isBusy() && rpmError<40){
                    if (outtake()) {
                        setPathState(1);
                    }
                }
                break;

            case 1:
//                move(false);
                if (!follower.isBusy()){
                    follower.followPath(grabPickup1);
                    setPathState(2);
                }

                break;
            case 2:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                if (follower.getPose().getY()<80){
                    setTurretPos(-7500);
                    intake();
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    follower.followPath(scorePickup1);
                    setPathState(99);
                }
                break;
            case 99:
                move(true);
//                if (follower.getPathCompletion()>0.3){
//                    robot.limiter.setPosition(Values.LIMITER_OPEN);
//                }
                if (!follower.isBusy()){
                    if (outtake()){
                        setPathState(3);
                    }

                }
                break;
            case 3:
//                move(false);
                if (follower.getPathCompletion()<0.6){
                    robot.intake.setPower(1);
                    robot.transfer.setPower(1);
                }
                follower.followPath(openGate);
                setPathState(4);

                break;
//                move(false);
//                follower.followPath(openGate);
//                setPathState(4);
//                break;
            case 4:
                if (follower.getPathCompletion()>0.7){
                    setTurretPos(-7400);
                }
                if (follower.getPathCompletion()>0.8){
                    intake();
                    follower.setMaxPower(0.3);
                }
                if (Values.counter==3 || pathTimer.getElapsedTimeSeconds()>3){
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup2);
                    setPathState(100);
                }
                break;
            case 100:
                move(true);
                if (!follower.isBusy()) {
                    if (outtake()){
                        setPathState(5);
                    }

                }
                break;
            case 5:
//                move(true);
//                if (follower.getPathCompletion()<0.6){
//                    keepShoot();
//                }else{
//                    intake();
//                }

                keepShoot();
                if (!follower.isBusy()){
                    follower.followPath(openGate2);
                    setPathState(6);
                }
                break;
//            case 100:
//                move(true);
//                if (!follower.isBusy()){
//                    if (outtake(0.5,2)) {
//                        setPathState(6);
//                    }
//                }break;
//            case 6:
//                if (follower.getPathCompletion()>0.5){
//                    intake();
//                }
//                if (follower.getPathCompletion()>0.8){
//                    follower.setMaxPower(0.3);
//                }
//                if (Values.counter==3 || pathTimer.getElapsedTimeSeconds()>3){
//                    follower.followPath(scorePickup3);
//                    setPathState(101);
//                }
//                break;

//            case 7:
//                if (follower.getPathCompletion()>0.7){
//                    intake();
//                }
//                if (!follower.isBusy()){
//                    follower.followPath(grabPickup2);
//                    setPathState(101);
//                }break;
            case 101:
                move(true);
                follower.setMaxPower(1);
                if (!follower.isBusy()){
                    if (outtake()){
                        setPathState(9);
                    }

                }break;
//            case 8:
//                move();
//                if (!follower.isBusy()){
//                    follower.followPath(toPickup4);
//                    setPathState(9);
//                }break;
            case 9:
//                move(false);
//                if (follower.getPathCompletion()<0.3){
//                    keepShoot();
//                }else{
//                    move(false);
//                }
                if (!follower.isBusy()){
                    follower.followPath(grabPickup2);
                    setPathState(10);
                }break;
            case 10:
                intake();
                setTurretPos(-4000);
                if (!follower.isBusy()){
                    follower.followPath(scorePickup4);
                    setPathState(102);
                }break;
            case 102:
                move(true);
                if (!follower.isBusy()){
                    if (outtake()){
                        setPathState(11);
                    }

                }break;
            case 11:
                move(false);
                if (!follower.isBusy()){
                    follower.followPath(grabPickup3);
                    setPathState(12);
                }break;
            case 12:
                if (follower.getPathCompletion()>0.6){
                    intake();
                    setTurretPos(-8500);
                }
                if (!follower.isBusy()){
                    follower.followPath(ScorePickup5);
                    setPathState(103);
                }break;
            case 103:
                move(true);
                if (!follower.isBusy()){
                    if (outtake()){
                        setPathState(104);
                    }

                }break;




//103
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
        ;


        if (Values.counter!=0) {
            robot.intake.setPower(1);
            robot.transfer.setPower(1);
        }
        if (Values.counter<Values.oldcounter && Values.counter==1){
            shootingTimer.resetTimer();
            return true;
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
        double avgFlywheel = (flywheelVel1 + flywheelVel2) / 2.0;
        rpmError = Math.abs(avgFlywheel - Values.flywheel_Values.flywheelTarget);
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
        followerDoneTimer = new Timer();
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
        Values.turretPos = methods.turretPID(turretEncoder, -13000);
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