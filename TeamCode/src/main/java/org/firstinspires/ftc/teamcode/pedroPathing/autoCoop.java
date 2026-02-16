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
    private final Pose startPose = new Pose(15.5, 115, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(49, 115, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1Pose = new Pose(17.9, 84.3, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup1 = new Pose(55.8,87.4);

    private final Pose gatePose = new Pose(16,69.7,Math.toRadians(180));
    private final Pose gateControl1 = new Pose(29.1,76);


    private final Pose scorePickup1Pose = new Pose(54,82.3,Math.toRadians(180));
    private final Pose controlScore1 = new Pose(46.5,73);

    private final Pose pickup2Pose = new Pose(10.9, 59.1, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup2 = new Pose(45.7,59.6);
    private final Pose gateControl2 = new Pose(26.5,63.2);

    private final Pose scorePickup2Pose = new Pose(54,82.3,Math.toRadians(180));
    private final Pose controlScore2 = new Pose(46.5,73);


    private final Pose pickup3Pose = new Pose(12.8, 35.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup3 = new Pose(47.6,36.5);

    private final Pose scorePickup3Pose = new Pose(48.7,87.8,Math.toRadians(180));

    private final Pose pickup4Pose = new Pose(8.6, 38.1, Math.toRadians(180));
    private final Pose control41 = new Pose(7.4,65.3);

    private final Pose scorePickup4Pose = new Pose(57.5,105.7,Math.toRadians(270));
    private final Pose leavePose = new Pose(25,70.9,Math.toRadians(180));
    private final Pose controlLeave = new Pose(37,71.3);

    private Path scorePreload;

    private PathChain grabPickup1, scorePickup1, gate1,grabPickup2, gate2,scorePickup2, grabPickup3, scorePickup3,grabPickup4,scorePickup4;

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
                .addPath(new BezierCurve(gatePose,controlScore1, scorePickup1Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
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

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePickup3Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePickup3Pose,control41,pickup4Pose))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup4Pose,scorePickup4Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();




    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>3){
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

                    follower.followPath(gate1);
                    setPathState(3);
                }
                break;
            case 3:
                move();
                if (!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
//                    if (outtake()) {
//                        setPathState(99);
//                    }
                    follower.followPath(scorePickup1);
                    setPathState(4);
                }
                break;
//            case 99:
//                move();
//                follower.followPath(grabPickup2);
//                setPathState(4);
//                break;
            case 4:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(grabPickup2);
                    setPathState(5);
                }
                break;
            case 5:
                move();
                if (!follower.isBusy()){
                    follower.followPath(gate2);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() &&pathTimer.getElapsedTimeSeconds()>2){
                    follower.followPath(scorePickup2);
                    setPathState(7);
                }
                break;
            case 7:
                if (follower.getPathCompletion()>0.5){
                    intake();
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(grabPickup3);
                    setPathState(8);
                }break;
            case 8:
                move();
                if (!follower.isBusy()){
                    follower.followPath(scorePickup3);
                    setPathState(9);
                }break;
            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(grabPickup4);
                    setPathState(10);
                }break;
            case 10:
                if (!follower.isBusy()){
                    follower.followPath(scorePickup4);
                    setPathState(11);
                }break;






        }
    }


    public void intake(){
        follower.setMaxPower(0.6);
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        robot.intake.setPower(1);
        robot.transfer.setPower(.5);

    }

    public void move(){
        follower.setMaxPower(1);
        robot.intake.setPower(0);
        robot.transfer.setPower(0);
    }
    public boolean outtake(){
        robot.limiter.setPosition(Values.LIMITER_OPEN);
        robot.intake.setPower(-1);
        robot.transfer.setPower(-0.5);
        if (pathTimer.getElapsedTimeSeconds()>2){
            return true;
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
        autonomousPathUpdate();

        Values.autonFollowerX = follower.getPose().getX();
        Values.autonFollowerY = follower.getPose().getY();
        Values.autonHeading = Math.toDegrees(follower.getHeading());

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
        intakePID = new Methods();
        transferPID = new Methods();
        flywheelPID = new Methods();
        methods = new Methods();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
//        double targetTurret = methods.AutoAim(follower.getPose(),robot.ll);
//        double turretEncoder = -robot.intake.getCurrentPosition();
//        Values.turretPos = methods.turretPID(turretEncoder, targetTurret+Values.turretOverride);
//        robot.turret1.setPosition(Values.turretPos);
//        robot.turret2.setPosition(Values.turretPos);
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