package org.firstinspires.ftc.teamcode.pedroPathing;
//we are skibidi, we are rizzlers
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Auton 12 Spike", group = "Auto")
public class AutonSpike12 extends OpMode {

    private Hardware robot;
    private Follower follower;
    private Methods intakePID,transferPID,flywheelPID,methods;
    private boolean pathStarted = false;
    private static final double CALLBACK_TIME = 10.0;

    private boolean emergencyTriggered = false;
    private boolean case15Completed = false;



    // PATH CHAINS
// BLUE
    public PathChain scorePreloadBlue;
    public PathChain toFirstChainBlue;
    public PathChain grabMiddleChainBlue;
    public PathChain openGatePathBlue;
    public PathChain scoreMiddleChainBluePath;
    public PathChain grabTopChainBluePath;
    public PathChain scoreTopChainBluePath;
    public PathChain toBottomChainBluePath;
    public PathChain grabBottomChainBluePath;
    public PathChain scoreBottomChainBluePath;

    // RED
    public PathChain scorePreloadRedChain;
    public PathChain toFirstChainRed;
    public PathChain grabMiddleChainRedPath;
    public PathChain openGatePathRed;
    public PathChain scoreMiddleChainRedPath;
    public PathChain grabTopChainRedPath;
    public PathChain scoreTopChainRedPath;
    public PathChain toBottomChainRedPath;
    public PathChain grabBottomChainRedPath;
    public PathChain scoreBottomChainRedPath;

    // FINAL SELECTED CHAINS
    private PathChain scorePreload;
    private PathChain toMiddleChain, grabMiddleChain, scoreMiddleChain;
    private PathChain  grabTopChain, scoreTopChain;
    private PathChain openGateChain;
    private PathChain toBottomChain, grabBottomChain, scoreBottomChain;


    private int pathState, actionState;
    private Timer pathTimer, actionTimer;

    public static Pose startingPose;

    // BLUE POSES+
    public static Pose startingPoseBlue = new Pose(33.3,136.7,Math.toRadians(180));
    public static Pose scorePreloadPoseBlue = new Pose(55.2,89,Math.toRadians(180));
    public static Pose toFirstBlue = new Pose(48,60,Math.toRadians(180));
    public static Pose controlToFirstBlue = new Pose(56.1,66.5);
    public static Pose grabMiddleBlue = new Pose(14.5,60,Math.toRadians(180));
    public static Pose openGateBlue = new Pose(15.9,74,Math.toRadians(180));
    public static Pose controlOpenGateBlue = new Pose(51.5,51.5);
    public static Pose scoreMiddleChainBlue = new Pose(53,84,Math.toRadians(180));
    public static Pose controlScoreMiddleBlue = new Pose(61.2,61.7);
    public static Pose grabTopChainBlue = new Pose(18,84,Math.toRadians(180));
    public static Pose scoreTopChainBlue = new Pose(53,84,Math.toRadians(180));
    public static Pose toBottomChainBlue = new Pose(43.75,36.2,Math.toRadians(180));
    public static Pose controlToBottomBlue = new Pose(52.5,35);
    public static Pose grabBottomChainBlue = new Pose(14,36.2,Math.toRadians(180));
    public static Pose scoreBottomChainBlue = new Pose(61.3,101.5,Math.toRadians(180));

    // RED POSES
    public static Pose startingPoseRed = mirrorPose(startingPoseBlue);
    public static Pose scorePreloadPoseRed = mirrorPose(scorePreloadPoseBlue);
    public static Pose toFirstRed = mirrorPose(toFirstBlue);
    public static Pose controlToFirstRed =mirrorPoint(controlToFirstBlue);
    public static Pose grabMiddleRed = mirrorPose(grabMiddleBlue);
    public static Pose openGateRed = mirrorPose(openGateBlue);
    public static Pose controlOpenGateRed = mirrorPoint(controlOpenGateBlue);
    public static Pose scoreMiddleChainRed = mirrorPose(scoreMiddleChainBlue);
    public static Pose controlScoreMiddleRed = mirrorPoint(controlScoreMiddleBlue);
    public static Pose grabTopChainRed = mirrorPose(grabTopChainBlue);
    public static Pose scoreTopChainRed = mirrorPose(scoreTopChainBlue);
    public static Pose toBottomChainRed = mirrorPose(toBottomChainBlue);
    public static Pose controlToBottomRed = mirrorPoint(controlToBottomBlue);
    public static Pose grabBottomChainRed = mirrorPose(grabBottomChainBlue);
    public static Pose scoreBottomChainRed = mirrorPose(scoreBottomChainBlue);

    private boolean isRed = false;





    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        pathTimer = new Timer();
        actionTimer = new Timer();

        robot = new Hardware(hardwareMap);
        intakePID = new Methods();
        transferPID = new Methods();
        flywheelPID = new Methods();
        methods = new Methods();
    }

    @Override
    public void init_loop() {

        if (gamepad1.aWasPressed()) {
            isRed = false;
            Values.team = Values.Team.BLUE;
            startingPose = startingPoseBlue;
        } else if (gamepad1.bWasPressed()) {
            isRed = true;
            Values.team = Values.Team.RED;
            startingPose = startingPoseRed;
        }

        if (startingPose == null) {
            Values.team = Values.Team.BLUE;
            startingPose = startingPoseBlue;
        }
        Values.turretOverride -= gamepad1.left_trigger/1000;
        Values.turretOverride += gamepad1.right_trigger/1000;
        robot.turret1.setPosition(Values.turretPos-Values.turretOverride);
        robot.turret2.setPosition(Values.turretPos-Values.turretOverride);
        robot.kicker.setPosition(Values.KICKER_DOWN);

        telemetry.addData("Team", Values.team);
        telemetry.addData("Starting Pose", startingPose);
        telemetry.addData("target",follower.getCurrentPath());
        telemetry.update();
    }

    @Override
    public void start() {

        isRed = (Values.team == Values.Team.RED);
        robot.ll.start();
        if (isRed){
            robot.ll.pipelineSwitch(1);
        }else{
            robot.ll.pipelineSwitch(2);
        }

        buildPaths();

        follower.setStartingPose(startingPose);

        scorePreload     = isRed ? scorePreloadRedChain     : scorePreloadBlue;

        toMiddleChain    = isRed ? toFirstChainRed          : toFirstChainBlue;
        grabMiddleChain  = isRed ? grabMiddleChainRedPath   : grabMiddleChainBlue;
        openGateChain    = isRed ? openGatePathRed              : openGatePathBlue;
        scoreMiddleChain = isRed ? scoreMiddleChainRedPath  : scoreMiddleChainBluePath;
        grabTopChain     = isRed ? grabTopChainRedPath      : grabTopChainBluePath;
        scoreTopChain    = isRed ? scoreTopChainRedPath     : scoreTopChainBluePath;

        toBottomChain    = isRed ? toBottomChainRedPath     : toBottomChainBluePath;
        grabBottomChain  = isRed ? grabBottomChainRedPath   : grabBottomChainBluePath;
        scoreBottomChain = isRed ? scoreBottomChainRedPath  : scoreBottomChainBluePath;



        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        Values.flywheel_Values.flywheelTarget=methods.flywheelControl(follower,robot.hood1);
        flywheelPID.velocity_PID(robot.flywheel1, robot.flywheel2,Values.flywheel_Values.flywheelTarget);
        intakePID.velocity_PID(robot.intake,Values.intake_Values.intakeTarget,"intake");
        transferPID.velocity_PID(robot.transfer,Values.transfer_Values.transferTarget,"transfer");
        robot.hood1.setPosition(methods.hoodControl(methods.getDist(follower.getPose()),robot.flywheel1,robot.flywheel2));
        methods.limelightCorrection(robot.ll,methods.getDist(follower.getPose()));
        robot.turret1.setPosition(methods.AutoAim(follower.getPose(),robot.ll));
        robot.turret2.setPosition(methods.AutoAim(follower.getPose(),robot.ll));

        Values.autonFollowerX = follower.getPose().getX();
        Values.autonFollowerY = follower.getPose().getY();

        telemetry.addData("Path State", pathState);
        telemetry.addData("drive error",follower.getDriveError());
        telemetry.addData("heading error",follower.getHeadingError());
        telemetry.addData("angular velocity",follower.getAngularVelocity());
        telemetry.addData("velocity",follower.getVelocity().getMagnitude());

        telemetry.update();
    }

    public void intake(){
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        robot.kicker.setPosition(Values.KICKER_DOWN);
        Values.intake_Values.intakeTarget=Values.intake_Values.intakeIntaking;
        Values.transfer_Values.transferTarget=Values.transfer_Values.transferIntake;
    }
    public void move(){
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        robot.kicker.setPosition(Values.KICKER_DOWN);
        Values.intake_Values.intakeTarget=Values.intake_Values.intakeHold*2;
        Values.transfer_Values.transferTarget=0;
    }
    public void moveNoIntake(){
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        robot.kicker.setPosition(Values.KICKER_DOWN);
        Values.intake_Values.intakeTarget=0;
        Values.transfer_Values.transferTarget=0;
    }
    public boolean shoot(){
        robot.limiter.setPosition(Values.LIMITER_OPEN);
        Values.intake_Values.intakeTarget=Values.intake_Values.intakeIntaking;
        Values.transfer_Values.transferTarget=Values.transfer_Values.transferUp;
        if (pathTimer.getElapsedTimeSeconds()>1.1){
            robot.kicker.setPosition(Values.KICKER_UP);
            return pathTimer.getElapsedTimeSeconds() > 1.4;
        }
        return false;
    }


    private void buildPaths() {

        // =======================
        // BLUE PATHS
        // =======================

        scorePreloadBlue = follower.pathBuilder()
                .addPath(new BezierLine(startingPoseBlue, scorePreloadPoseBlue))
                .setLinearHeadingInterpolation(
                        startingPoseBlue.getHeading(),
                        scorePreloadPoseBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        toFirstChainBlue = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePreloadPoseBlue,
                        controlToFirstBlue,
                        toFirstBlue))
                .setLinearHeadingInterpolation(
                        scorePreloadPoseBlue.getHeading(),
                        toFirstBlue.getHeading())
                .setNoDeceleration()
                .setHeadingConstraint(0.3)
                .build();

        grabMiddleChainBlue = follower.pathBuilder()
                .addPath(new BezierLine(toFirstBlue, grabMiddleBlue))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();
        openGatePathBlue = follower.pathBuilder()
                .addPath(new BezierCurve(grabMiddleBlue,controlOpenGateBlue,openGateBlue))
                .setLinearHeadingInterpolation(grabMiddleBlue.getHeading(),openGateBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        scoreMiddleChainBluePath = follower.pathBuilder()
                .addPath(new BezierCurve(openGateBlue, controlScoreMiddleBlue,scoreMiddleChainBlue))
                .setLinearHeadingInterpolation(
                        grabMiddleBlue.getHeading(),
                        scoreMiddleChainBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();


        grabTopChainBluePath = follower.pathBuilder()
                .addPath(new BezierLine(scoreMiddleChainBlue, grabTopChainBlue))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();

        scoreTopChainBluePath = follower.pathBuilder()
                .addPath(new BezierLine(grabTopChainBlue, scoreTopChainBlue))
                .setLinearHeadingInterpolation(
                        grabTopChainBlue.getHeading(),
                        scoreTopChainBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        toBottomChainBluePath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scoreTopChainBlue,
                        controlToBottomBlue,
                        toBottomChainBlue))
                .setLinearHeadingInterpolation(
                        scoreTopChainBlue.getHeading(),
                        toBottomChainBlue.getHeading())
                .setNoDeceleration()
                .setHeadingConstraint(0.3)
                .build();

        grabBottomChainBluePath = follower.pathBuilder()
                .addPath(new BezierLine(toBottomChainBlue, grabBottomChainBlue))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();

        scoreBottomChainBluePath = follower.pathBuilder()
                .addPath(new BezierLine(grabBottomChainBlue, scoreBottomChainBlue))
                .setLinearHeadingInterpolation(
                        grabBottomChainBlue.getHeading(),
                        scoreBottomChainBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();


        // =======================
        // RED PATHS
        // =======================

        scorePreloadRedChain = follower.pathBuilder()
                .addPath(new BezierLine(startingPoseRed, scorePreloadPoseRed))
                .setLinearHeadingInterpolation(
                        startingPoseRed.getHeading(),
                        scorePreloadPoseRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        toFirstChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePreloadPoseRed,
                        controlToFirstRed,
                        toFirstRed))
                .setLinearHeadingInterpolation(
                        scorePreloadPoseRed.getHeading(),
                        toFirstRed.getHeading())
                .setHeadingConstraint(0.3)
                .setNoDeceleration()
                .build();

        grabMiddleChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(toFirstRed, grabMiddleRed))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();
        openGatePathRed = follower.pathBuilder()
                .addPath(new BezierCurve(grabMiddleRed,controlOpenGateRed,openGateRed))
                .setLinearHeadingInterpolation(grabMiddleRed.getHeading(),openGateRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        scoreMiddleChainRedPath = follower.pathBuilder()
                .addPath(new BezierCurve(openGateRed, controlScoreMiddleRed,scoreMiddleChainRed))
                .setLinearHeadingInterpolation(
                        grabMiddleRed.getHeading(),
                        scoreMiddleChainRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();


        grabTopChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(scoreMiddleChainRed, grabTopChainRed))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();

        scoreTopChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(grabTopChainRed, scoreTopChainRed))
                .setLinearHeadingInterpolation(
                        grabTopChainRed.getHeading(),
                        scoreTopChainRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        toBottomChainRedPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scoreTopChainRed,
                        controlToBottomRed,
                        toBottomChainRed))
                .setLinearHeadingInterpolation(
                        scoreTopChainRed.getHeading(),
                        toBottomChainRed.getHeading())
                .setHeadingConstraint(0.3)
                .setNoDeceleration()
                .build();

        grabBottomChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(toBottomChainRed, grabBottomChainRed))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();

        scoreBottomChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(grabBottomChainRed, scoreBottomChainRed))
                .setLinearHeadingInterpolation(
                        grabBottomChainRed.getHeading(),
                        scoreBottomChainRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();

    }


    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                move();
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    nextPath();
                }
                break;

            case 1:
                move();
                runPath(scorePreload);
                break;

            case 2:
            case 7:
            case 11:
            case 16:
                if (Math.abs(Values.flywheel_Values.flywheelTarget-robot.flywheel1.getVelocity())<50 && follower.getAngularVelocity()<.5 && follower.getVelocity().getMagnitude()<1) {
                    nextPath();
                }
                break;
            case 3:
            case 8:
            case 12:
            case 17:
                if (shoot()){
                    nextPath();
                }break;

            // grab middle & open gate
            case 4:
                intake();
                runPath(grabMiddleChain);
                break;

            case 5:
                move();
                runPath(openGateChain);
                break;
            case 6:
                move();
                runPath(scoreMiddleChain);
                break;
            // grab top
            case 9:
                intake();
                runPath(grabTopChain);
                break;
            case 10:
                move();
                runPath(scoreTopChain);
                break;
            //grab bottom
            case 13:
                move();
                runPath(toBottomChain);
                break;
            case 14:
                intake();
                runPath(grabBottomChain);
                break;
            case 15:
                move();
                runPath(scoreBottomChain);
                break;





            default:
                move();
                break;
        }
    }


    private void nextPath() {
        pathState++;
        pathTimer.resetTimer();
        pathStarted=false;
    }

    private void setPathState(int p) {
        pathState = p;
        pathTimer.resetTimer();
        actionState = 0;
    }

    public static Pose mirrorPose(Pose p) {
        return new Pose(
                144 - p.getX(),
                p.getY(),
                Math.PI - p.getHeading()
        );
    }

    public static Pose mirrorPoint(Pose p) {
        return new Pose(
                144 - p.getX(),
                p.getY()
        );
    }
    private void runPath(PathChain path) {

        if (!pathStarted) {
            follower.followPath(path,true);
            pathStarted = true;
            pathTimer.resetTimer();

        }
        if (pathState==5){
            follower.setMaxPower(0.7);
            if (pathTimer.getElapsedTimeSeconds()>4){
                nextPath();
            }
        }else if (pathState==4){
            follower.setMaxPower(0.7);
        }else{
            follower.setMaxPower(1);
        }
//        if (follower.getDistanceRemaining()<10){
//            follower.setMaxPower(0.8);
//        }else{
//            follower.setMaxPower(1);
//        }

        boolean headingGood =
                Math.abs(follower.getHeadingError()) <follower.getCurrentPath().getPathEndHeadingConstraint();
        if (follower.atParametricEnd() && headingGood || follower.isRobotStuck() || pathTimer.getElapsedTimeSeconds()>7 || !follower.isBusy()) {
            pathStarted = false;
            pathTimer.resetTimer();
            nextPath();
        }

    }


}
