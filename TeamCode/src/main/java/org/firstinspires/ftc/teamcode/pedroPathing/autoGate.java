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

@Autonomous(name = "Gate", group = "Blue")
public class autoGate extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Hardware robot;
    private Methods intakePID,transferPID,flywheelPID,methods;

    private int pathState;
    private final Pose startPose = new Pose(15.5, 113.5, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(49, 115, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1Pose = new Pose(12.7, 59, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup1 = new Pose(48,58.1);

    private final Pose scorePickup1Pose = new Pose(50,81.3,Math.toRadians(180));

    private final Pose tapGatePose = new Pose(16,70.7,Math.toRadians(180));
    private final Pose tunnelPose = new Pose(12.3,56,Math.toRadians(120));
    private final Pose controlTunnel = new Pose(18.9,57.8);

    private final Pose scoreGatePose = new Pose(50,81.3,Math.toRadians(180));
    private final Pose scoreGatePose2 = new Pose(50,84,Math.toRadians(180));

    private final Pose pickup2Pose = new Pose(24,84,Math.toRadians(180));

    private final Pose scorePickup2Pose = new Pose(50,84,Math.toRadians(180));

    private final Pose pickup3Pose = new Pose(20.9, 37.2, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose controlPickup3 = new Pose(40.9,38.25);

    private final Pose scorePickup3Pose = new Pose(60.07,102.1,Math.toRadians(180));

    private final Pose controlScore4Pose = new Pose(6.9,64.9);
    private Path scorePreload;

    private PathChain grabPickup1, scorePickup1, tapGate, tunnel, scoreGate1, scoreGate2, grabPickup2, scorePickup2,grabPickup3,scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(scorePose.getHeading());

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,controlPickup1, pickup1Pose))
                .setTangentHeadingInterpolation()
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePickup1Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        tapGate = follower.pathBuilder()
                .addPath(new BezierLine(scorePickup1Pose,tapGatePose))
                .setTangentHeadingInterpolation()
//                .setNoDeceleration()
                .build();

        tunnel = follower.pathBuilder()
                .addPath(new BezierCurve(tapGatePose,controlTunnel,tunnelPose))
                .setLinearHeadingInterpolation(tapGatePose.getHeading(), tunnelPose.getHeading())
                .build();

        scoreGate1 = follower.pathBuilder()
                .addPath(new BezierLine(tunnelPose,scoreGatePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        scoreGate2 = follower.pathBuilder()
                .addPath(new BezierLine(tunnelPose,scoreGatePose2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scoreGatePose2,pickup2Pose))
                .setTangentHeadingInterpolation()
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose,scorePickup2Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePickup2Pose,controlPickup3,pickup3Pose))
                .setTangentHeadingInterpolation()
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose,scorePickup3Pose))
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
//                if (follower.getPathCompletion()>0.5){
//                    intake();
//                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    follower.followPath(scorePickup1);
                    setPathState(3);
                }
                break;
            case 3:
            case 6:
                if (!follower.isBusy()){
                    follower.followPath(tapGate);
                    nextPath();
                }break;
            case 4:
            case 7:
                if (follower.getPathCompletion()>0.8){
                    follower.setMaxPower(.5);
                }
                if (!follower.isBusy() ){
                    follower.setMaxPower(1);
                    follower.followPath(tunnel);
                    nextPath();
                }break;
            case 5:
                if (!follower.isBusy()){
                    follower.followPath(scoreGate1);
                    setPathState(6);
                }
            case 8:
                if (!follower.isBusy()){
                    follower.followPath(scoreGate2);
                    setPathState(9);
                }break;
            case 9:
                if (!follower.isBusy()){
                    follower.followPath(grabPickup2);
                    setPathState(10);
                }break;
            case 10:
                if (!follower.isBusy()){
                    follower.followPath(scorePickup2);
                    setPathState(11);
                }break;
            case 11:
                if (!follower.isBusy()){
                    follower.followPath(grabPickup3);
                    setPathState(12);
                }break;
            case 12:
                if (!follower.isBusy()){
                    follower.followPath(scorePickup3);
                    setPathState(13);
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