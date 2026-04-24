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

@Autonomous(name = "Blue 18", group = "Blue")
public class autoClose extends OpMode {

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
    private final Pose scorePose = new Pose(49.5, 102.5, Math.toRadians(265));
    private final Pose pickup1Pose = new Pose(12, 57.7, Math.toRadians(180)); // -140 Highest (First  Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup1 = new Pose(46,63.9);
//    private final Pose controlPickup1_2 = new Po\e(39.6,69.6);

    private final Pose scorePoses = new Pose(54.6,75,Math.toRadians(210));

    private final Pose openGatePose = new Pose(16,70,Math.toRadians(180));
    private final Pose controlGate = new Pose(35.7,70);
    private final Pose gateBackPose = new Pose(11,55,Math.toRadians(160));
    private final double rotDeg = Math.toRadians(120);
//    private final Pose controlGate = new Pose(29.2,65.2);




    private final Pose pickup2Pose = new Pose(14, 86.4, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose controlPickup2 = new Pose(46.5,60.7);


    private final Pose scorePickup5Pose = new Pose(53.4,112.7,Math.toRadians(180));


//    private final Pose toLoadingPose = new Pose(7,35.5,Math.toRadians(225));
//    private final Pose pickup4Pose = new Pose(9,50,Math.toRadians(270));
//    private final Pose controlPickup4 = new Pose(36.4,50.5);
//
////    private final Pose scorePickup4Pose = new Pose(51,117.6,Math.toRadians(180));
//    private final Pose controlScore4Pose = new Pose(38.8,76.6);
    private Path scorePreload;

    private PathChain grabPickup1, scorePickup1, openGate,gateBack,scoreGates,grabPickup2,scorePickup4;

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
                .addPath(new BezierLine(pickup1Pose,scorePoses))
                .setTranslationalConstraint(2)
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePoses, controlGate,openGatePose))
                .setHeadingConstraint(0.5)
                .setTangentHeadingInterpolation()
                .build();

        gateBack = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose,gateBackPose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), gateBackPose.getHeading())
                .setTranslationalConstraint(2)
                .setHeadingConstraint(0.5)
                .build();





        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreGates = follower.pathBuilder()
                .addPath(new BezierLine(gateBackPose, scorePoses))
                .setConstantHeadingInterpolation(scorePoses.getHeading())
                .build();



        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoses, pickup2Pose))
                .setTangentHeadingInterpolation()
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePickup5Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();






    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setTurretPos(-12500);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                robot.limiter.setPosition(Values.LIMITER_OPEN);
//                move(false);
                if (!follower.isBusy() && rpmError<40){
                    if (instantOuttake()) {
                        setPathState(2);
                    }
                }
                break;

            case 2:
//                move(false);
                if (!follower.isBusy()){
                    follower.followPath(grabPickup1);
                    setPathState(3);
                }

                break;
            case 3:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                if (follower.getPose().getY()<80){
                    setTurretPos(-8100);
                    intake();
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    follower.followPath(scorePickup1);
                    setPathState(4);
                }
                break;
            case 4:
                move(true);
//                if (follower.getPathCompletion()>0.3){
//                    robot.limiter.setPosition(Values.LIMITER_OPEN);
//                }
                if (!follower.isBusy()){
                    if (instantOuttake()){
                        setPathState(5);
                    }

                }
                break;
            case 5:
//                move(false);
                if (follower.getPathCompletion()<0.6){
                    robot.intake.setPower(1);
                    robot.transfer.setPower(1);
                }
                follower.followPath(openGate);
                setPathState(100);

                break;
            case 100:
                if (follower.getPathCompletion()>0.6){
                    intake();
                }
                if (follower.getPathCompletion()>0.9){
                    follower.followPath(gateBack);
                    setPathState(200);
                }
                break;
            case 200:
                if (follower.getPathCompletion()>0.7){
                    setTurretPos(-8500);

                }
                if (!follower.isBusy()){
                    follower.turn(gateBackPose.getHeading()-rotDeg,false);
                    setPathState(6);
                }
                break;
            case 6:

                if (Values.counter==3 || pathTimer.getElapsedTimeSeconds()>0.8){
                    follower.followPath(scoreGates);
                    setPathState(7);
                }
                break;
            case 7:
                move(true);
                if (!follower.isBusy()) {
                    if (outtake()){
                        setPathState(8);
                    }

                }
                break;
            case 8:
                if (!follower.isBusy()){
                    follower.followPath(openGate);
                    setPathState(101);
                }
                break;

            case 101:
                if (follower.getPathCompletion()>0.6){
                    intake();
                }
                if (follower.getPathCompletion()>0.9){
                    follower.followPath(gateBack);
                    setPathState(201);
                }
                break;
            case 201:
                if (!follower.isBusy()){
                    follower.turn(gateBackPose.getHeading()-rotDeg,false);
                    setPathState(9);
                }
                break;
            case 9:
                if (Values.counter==3 || pathTimer.getElapsedTimeSeconds()>0.8){
                    follower.followPath(scoreGates);
                    setPathState(10);
                }
                break;
            case 10:
                move(true);
                if (!follower.isBusy()) {
                    if (outtake()){
                        setPathState(11);
                    }

                }
                break;
            case 11:
                if (!follower.isBusy()){
                    follower.followPath(openGate);
                    setPathState(102);
                }
                break;
            case 102:
                if (follower.getPathCompletion()>0.6){
                    intake();
                }
                if (follower.getPathCompletion()>0.9){
                    follower.followPath(gateBack);
                    setPathState(203);
                }
                break;
            case 202:
                if (!follower.isBusy()){
                    follower.turn(gateBackPose.getHeading()-rotDeg,false);
                    setPathState(12);
                }
                break;

            case 12:

                if (Values.counter==3 || pathTimer.getElapsedTimeSeconds()>0.8){
                    follower.followPath(scoreGates);
                    setPathState(13);
                }
                break;
            case 13:
                move(true);
                if (!follower.isBusy()) {
                    if (outtake()){
                        setPathState(14);
                    }

                }
                break;
            case 14:
                if (!follower.isBusy()){
                    follower.followPath(openGate);
                    setPathState(103);
                }
                break;
            case 103:
                if (follower.getPathCompletion()>0.6){
                    intake();
                }
                if (follower.getPathCompletion()>0.9){
                    follower.followPath(gateBack);
                    setPathState(203);
                }
                break;
            case 203:
                if (!follower.isBusy()){
                    follower.turn(gateBackPose.getHeading()-rotDeg,false);
                    setPathState(15);
                }
                break;

            case 15:

                if (Values.counter==3 || pathTimer.getElapsedTimeSeconds()>0.8){
                    follower.followPath(scoreGates);
                    setPathState(16);
                }
                break;

            case 16:
                move(true);
                if (!follower.isBusy()) {
                    if (outtake()){
                        setPathState(17);
                    }

                }
                break;
            case 17:
                if (!follower.isBusy()){
                    follower.followPath(grabPickup2);
                    setPathState(18);
                }break;
            case 18:
                if (follower.getPathCompletion()>0.3) {
                    intake();
                    setTurretPos(-6800);
                }
                if (!follower.isBusy()){
                    follower.followPath(scorePickup4);
                    setPathState(19);
                }break;
            case 19:
                move(true);
                if (!follower.isBusy()){
                    if (outtake()){
                        setPathState(20);
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
        if (Values.counter<Values.oldcounter || Values.counter==1){
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
        Values.turretPos = methods.turretPID(turretEncoder, -12800);
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