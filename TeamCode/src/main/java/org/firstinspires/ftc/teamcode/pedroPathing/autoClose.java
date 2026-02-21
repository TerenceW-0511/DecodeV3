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

@Autonomous(name = "Close 15 Blue", group = "Blue")
public class autoClose extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Hardware robot;
    private Methods intakePID,transferPID,flywheelPID,methods;

    private int pathState;
    private final Pose startPose = new Pose(15.5, 113.5, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(49, 115, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1Pose = new Pose(17.3, 84.3, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup1 = new Pose(49.5,87.8);

    private final Pose scorePickup1Pose = new Pose(50,84.3,Math.toRadians(180));

    private final Pose pickup2Pose = new Pose(6, 59.1, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup2 = new Pose(46.5,60.7);

    private final Pose openGatePose = new Pose(14,71,Math.toRadians(180));
    private final Pose controlGate = new Pose(29.2,65.2);

    private final Pose scorePickup2Pose = new Pose(50.2,85,Math.toRadians(180));

    private final Pose pickup3Pose = new Pose(6, 35.5, Math.toRadians(225)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup3 = new Pose(48.5,35.2);

    private final Pose scorePickup3Pose = new Pose(50,85,Math.toRadians(225));

    private final Pose toLoadingPose = new Pose(7,35.5,Math.toRadians(225));
    private final Pose pickup4Pose = new Pose(9,50,Math.toRadians(270));
    private final Pose controlPickup4 = new Pose(36.4,50.5);

    private final Pose scorePickup4Pose = new Pose(51,117.6,Math.toRadians(180));
    private final Pose controlScore4Pose = new Pose(38.8,76.6);
    private Path scorePreload;

    private PathChain grabPickup1, scorePickup1, grabPickup2, openGate,scorePickup2, grabPickup3, scorePickup3,toPickup4,grabPickup4,scorePickup4;

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

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePickup1Pose))
                .setConstantHeadingInterpolation(scorePickup1Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePickup1Pose,controlPickup2, pickup2Pose))
                .setTangentHeadingInterpolation()
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose,controlGate, openGatePose))
                .setConstantHeadingInterpolation(openGatePose.getHeading())
                .build();


        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, scorePickup2Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePickup2Pose, controlPickup3,pickup3Pose))
                .setTangentHeadingInterpolation()
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePickup3Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePickup3Pose.getHeading())
                .build();

//        toPickup4 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePickup3Pose,toLoadingPose))
//                .setConstantHeadingInterpolation(toLoadingPose.getHeading())
//                .build();
        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePickup3Pose,controlPickup4,pickup4Pose))
                .setTangentHeadingInterpolation()
                .build();
        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup4Pose,controlScore4Pose,scorePickup4Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();





    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                follower.followPath(scorePreload);
                setPathState(98);
                break;
            case 98:
                move(true);
                if (!follower.isBusy()){
                    if (outtake(0.5,2)) {
                        setPathState(1);
                    }
                }
                break;

            case 1:
                move(false);
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
                if (follower.getPathCompletion()>0.5){
                    intake();
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    follower.followPath(scorePickup1);
                    setPathState(3);
                }
                break;
            case 3:
                move(true);
                if (!follower.isBusy()){
                    if (outtake(0.5,2)) {
                        setPathState(99);
                    }
                }
                break;
            case 99:
                move(false);
                follower.followPath(grabPickup2);
                setPathState(4);
                break;
            case 4:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(openGate);
                    setPathState(5);
                }
                break;
            case 5:
                gate();
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2){
                    follower.followPath(scorePickup2);
                    setPathState(100);
                }
                break;
            case 100:
                move(true);
                if (!follower.isBusy()){
                    if (outtake(0.5,2)) {
                        setPathState(6);
                    }
                }break;
            case 6:
                move(false);
                if (!follower.isBusy()){
                    follower.followPath(grabPickup3);
                    setPathState(7);
                }
                break;
            case 7:
                if (follower.getPathCompletion()>0.7){
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(scorePickup3);
                    setPathState(101);
                }break;
            case 101:
                move(true);
                if (!follower.isBusy()){
                    if (outtake(0.5,2)) {
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
                move(false);
                if (!follower.isBusy()){
                    follower.followPath(grabPickup4);
                    setPathState(10);
                }break;
            case 10:
                if (follower.getPathCompletion()>0.8){
                    intake();
                }
                if (!follower.isBusy()){
                    follower.followPath(scorePickup4);
                    setPathState(102);
                }break;
            case 102:
                move(true);
                if (!follower.isBusy()){
                    if (outtake(0.5,2)) {
                        setPathState(103);
                    }
                }break;





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
            if (follower.getPathCompletion()>0.7){
                robot.limiter.setPosition(Values.LIMITER_OPEN);
            }else{
                robot.limiter.setPosition(Values.LIMITER_CLOSE);
            }
            if (follower.getPathCompletion()<.2){
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
        robot.hood1.setPosition(1);
        Values.turretPos = methods.turretPID(turretEncoder, -8000);
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