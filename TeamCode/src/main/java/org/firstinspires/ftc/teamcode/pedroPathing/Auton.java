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

@Autonomous(name = "Auton", group = "Auto")
public class Auton extends OpMode {

    private Hardware robot;
    private Follower follower;
    private boolean pathStarted = false;


    // PATH CHAINS
    public PathChain scorePreloadBlue, scorePreloadRedChain;
    public PathChain toFirstChainBlue, toFirstChainRed;
    public PathChain grabMiddleChainBlue, grabMiddleChainRedPath;
    public PathChain scoreMiddleChainBluePath, scoreMiddleChainRedPath;
    public PathChain gateIntakeChainBlue, gateIntakeChainRed;
    public PathChain scoreGateChainBlue, scoreGateChainRed;
    public PathChain toTopChainBluePath, toTopChainRedPath;
    public PathChain grabTopChainBluePath, grabTopChainRedPath;
    public PathChain scoreTopChainBluePath, scoreTopChainRedPath;
    public PathChain toBottomChainBluePath, toBottomChainRedPath;
    public PathChain grabBottomChainBluePath, grabBottomChainRedPath;
    public PathChain scoreBottomChainBluePath, scoreBottomChainRedPath;
    public PathChain leaveBlue, leaveRed;


    private int pathState, actionState;
    private Timer pathTimer, actionTimer;

    public static Pose startingPose;

    // BLUE POSES
    public static Pose startingPoseBlue = new Pose(18,120,Math.toRadians(144));
    public static Pose scorePreloadPoseBlue = new Pose(55.2,89,Math.toRadians(144));
    public static Pose toFirstBlue = new Pose(44.8,60.3,Math.toRadians(180));
    public static Pose controlToFirstBlue = new Pose(56.1,66.5);
    public static Pose grabMiddleBlue = new Pose(14.2,59.4,Math.toRadians(180));
    public static Pose scoreMiddleChainBlue = new Pose(53,81,Math.toRadians(180));
    public static Pose gateIntakeBlue = new Pose(12.5,60,Math.toRadians(140));
    public static Pose scoreGateBlue = new Pose(54,82,Math.toRadians(140));
    public static Pose toTopChainBlue = new Pose(45.2,84.3,Math.toRadians(180));
    public static Pose controlToTopBlue = new Pose(40.6,67.4);
    public static Pose grabTopChainBlue = new Pose(16.7,84.1,Math.toRadians(180));
    public static Pose scoreTopChainBlue = new Pose(55,88.1,Math.toRadians(270));
    public static Pose controlScoreTopBlue = new Pose(57.8,91.9);
    public static Pose toBottomChainBlue = new Pose(43.75,36.2,Math.toRadians(180));
    public static Pose controlToBottomBlue = new Pose(52.5,30.77);
    public static Pose grabBottomChainBlue = new Pose(16.5,35.8,Math.toRadians(180));
    public static Pose scoreBottomChainBlue = new Pose(52.7,80.8,Math.toRadians(180));
    public static Pose leavePathBlue = new Pose(21.8,70.3,Math.toRadians(180));

    // RED POSES
    public static Pose startingPoseRed = mirrorPose(startingPoseBlue);
    public static Pose scorePreloadPoseRed = mirrorPose(scorePreloadPoseBlue);
    public static Pose toFirstRed = mirrorPose(toFirstBlue);
    public static Pose controlToFirstRed = mirrorPoint(controlToFirstBlue);
    public static Pose grabMiddleChainRed = mirrorPose(grabMiddleBlue);
    public static Pose scoreMiddleChainRed = mirrorPose(scoreMiddleChainBlue);
    public static Pose gateIntakeRed = mirrorPose(gateIntakeBlue);
    public static Pose scoreGateRed = mirrorPose(scoreGateBlue);
    public static Pose toTopChainRed = mirrorPose(toTopChainBlue);
    public static Pose controlToTopRed = mirrorPoint(controlToTopBlue);
    public static Pose grabTopChainRed = mirrorPose(grabTopChainBlue);
    public static Pose scoreTopChainRed = mirrorPose(scoreTopChainBlue);
    public static Pose controlScoreTopRed = mirrorPoint(controlScoreTopBlue);
    public static Pose toBottomChainRed = mirrorPose(toBottomChainBlue);
    public static Pose controlToBottomRed = mirrorPoint(controlToBottomBlue);
    public static Pose grabBottomChainRed = mirrorPose(grabBottomChainBlue);
    public static Pose scoreBottomChainRed = mirrorPose(scoreBottomChainBlue);
    public static Pose leavePathRed = mirrorPose(leavePathBlue);

    private boolean isRed = true;

    //final chains
    private PathChain scorePreload;
    private PathChain toFirstChain;
    private PathChain grabMiddleChain, scoreMiddleChain;
    private PathChain gateIntakeChain, scoreGateChain;
    private PathChain toTopChain, grabTopChain, scoreTopChain;
    private PathChain toBottomChain, grabBottomChain, scoreBottomChain;
    private PathChain leave;

    public void intake(){
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        Values.intake_Values.intakeTarget=Values.intake_Values.intakeIntaking;
        Values.transfer_Values.transferTarget=Values.transfer_Values.transferIntake;
    }
    public void move(){
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        Values.intake_Values.intakeTarget=Values.intake_Values.intakeHold;
        Values.transfer_Values.transferTarget=0;
    }
    public void shoot(){
        robot.limiter.setPosition(Values.LIMITER_OPEN);
        Values.intake_Values.intakeTarget=Values.intake_Values.intakeIntaking;
        Values.transfer_Values.transferTarget=Values.transfer_Values.transferUp;
    }



    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        pathTimer = new Timer();
        actionTimer = new Timer();
    }

    @Override
    public void init_loop() {

        if (gamepad1.aWasPressed()) {
            Values.team = Values.Team.BLUE;
            startingPose = startingPoseBlue;
        } else if (gamepad1.bWasPressed()) {
            Values.team = Values.Team.RED;
            startingPose = startingPoseRed;
        }

        if (startingPose == null) {
            Values.team = Values.Team.BLUE;
            startingPose = startingPoseBlue;
        }

        telemetry.addData("Team", Values.team);
        telemetry.addData("Starting Pose", startingPose);
        telemetry.addData("target",follower.getCurrentPath());
        telemetry.update();
    }

    @Override
    public void start() {

        isRed = (Values.team == Values.Team.RED);

        buildPaths();
        //what the freak is this bruh (i dont program) nvm i got it
        follower.setStartingPose(startingPose);

        scorePreload     = isRed ? scorePreloadRedChain     : scorePreloadBlue;
        toFirstChain     = isRed ? toFirstChainRed          : toFirstChainBlue;
        grabMiddleChain  = isRed ? grabMiddleChainRedPath   : grabMiddleChainBlue;
        scoreMiddleChain = isRed ? scoreMiddleChainRedPath  : scoreMiddleChainBluePath;
        gateIntakeChain  = isRed ? gateIntakeChainRed       : gateIntakeChainBlue;
        scoreGateChain   = isRed ? scoreGateChainRed        : scoreGateChainBlue;
        toTopChain       = isRed ? toTopChainRedPath        : toTopChainBluePath;
        grabTopChain     = isRed ? grabTopChainRedPath      : grabTopChainBluePath;
        scoreTopChain    = isRed ? scoreTopChainRedPath     : scoreTopChainBluePath;
        toBottomChain    = isRed ? toBottomChainRedPath     : toBottomChainBluePath;
        grabBottomChain  = isRed ? grabBottomChainRedPath   : grabBottomChainBluePath;
        scoreBottomChain = isRed ? scoreBottomChainRedPath  : scoreBottomChainBluePath;
        leave            = isRed ? leaveRed                 : leaveBlue;

        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("drive error",follower.getDriveError());
        telemetry.addData("heading error",follower.getHeadingError());
        telemetry.update();
    }

    private void buildPaths() {

        // BLUE
        scorePreloadBlue = follower.pathBuilder()
                .addPath(new BezierLine(startingPoseBlue, scorePreloadPoseBlue))
                .setLinearHeadingInterpolation(
                        startingPoseBlue.getHeading(),
                        scorePreloadPoseBlue.getHeading())
                .build();

        toFirstChainBlue = follower.pathBuilder()
                .addPath(new BezierCurve(scorePreloadPoseBlue, controlToFirstBlue, toFirstBlue))
                .setLinearHeadingInterpolation(
                        scorePreloadPoseBlue.getHeading(),
                        toFirstBlue.getHeading())
                .build();

        grabMiddleChainBlue = follower.pathBuilder()
                .addPath(new BezierLine(toFirstBlue, grabMiddleBlue))
                .setTangentHeadingInterpolation()
                .build();

        scoreMiddleChainBluePath = follower.pathBuilder()
                .addPath(new BezierLine(grabMiddleBlue, scoreMiddleChainBlue))
                .setLinearHeadingInterpolation(
                        grabMiddleBlue.getHeading(),
                        scoreMiddleChainBlue.getHeading())
                .build();

        gateIntakeChainBlue = follower.pathBuilder()
                .addPath(new BezierLine(scoreMiddleChainBlue, gateIntakeBlue))
                .setLinearHeadingInterpolation(
                        scoreMiddleChainBlue.getHeading(),
                        gateIntakeBlue.getHeading())
                .build();

        scoreGateChainBlue = follower.pathBuilder()
                .addPath(new BezierLine(gateIntakeBlue, scoreGateBlue))
                .setLinearHeadingInterpolation(
                        gateIntakeBlue.getHeading(),
                        scoreGateBlue.getHeading())
                .build();

        toTopChainBluePath = follower.pathBuilder()
                .addPath(new BezierCurve(scoreGateBlue, controlToTopBlue, toTopChainBlue))
                .setLinearHeadingInterpolation(
                        scoreGateBlue.getHeading(),
                        toTopChainBlue.getHeading())
                .build();

        grabTopChainBluePath = follower.pathBuilder()
                .addPath(new BezierLine(toTopChainBlue, grabTopChainBlue))
                .setTangentHeadingInterpolation()
                .build();

        scoreTopChainBluePath = follower.pathBuilder()
                .addPath(new BezierCurve(grabTopChainBlue, controlScoreTopBlue, scoreTopChainBlue))
                .setLinearHeadingInterpolation(
                        grabTopChainBlue.getHeading(),
                        scoreTopChainBlue.getHeading())
                .build();

        toBottomChainBluePath = follower.pathBuilder()
                .addPath(new BezierCurve(scoreTopChainBlue, controlToBottomBlue, toBottomChainBlue))
                .setLinearHeadingInterpolation(
                        scoreTopChainBlue.getHeading(),
                        toBottomChainBlue.getHeading())
                .build();

        grabBottomChainBluePath = follower.pathBuilder()
                .addPath(new BezierLine(toBottomChainBlue, grabBottomChainBlue))
                .setTangentHeadingInterpolation()
                .build();

        scoreBottomChainBluePath = follower.pathBuilder()
                .addPath(new BezierLine(grabBottomChainBlue, scoreBottomChainBlue))
                .setLinearHeadingInterpolation(
                        grabBottomChainBlue.getHeading(),
                        scoreBottomChainBlue.getHeading())
                .build();

        leaveBlue = follower.pathBuilder()
                .addPath(new BezierLine(scoreBottomChainBlue, leavePathBlue))
                .setTangentHeadingInterpolation()
                .build();

        // RED
        scorePreloadRedChain = follower.pathBuilder()
                .addPath(new BezierLine(startingPoseRed, scorePreloadPoseRed))
                .setLinearHeadingInterpolation(
                        startingPoseRed.getHeading(),
                        scorePreloadPoseRed.getHeading())
                .build();

        toFirstChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(scorePreloadPoseRed, controlToFirstRed, toFirstRed))
                .setLinearHeadingInterpolation(
                        scorePreloadPoseRed.getHeading(),
                        toFirstRed.getHeading())
                .build();

        grabMiddleChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(toFirstRed, grabMiddleChainRed))
                .setTangentHeadingInterpolation()
                .build();

        scoreMiddleChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(grabMiddleChainRed, scoreMiddleChainRed))
                .setLinearHeadingInterpolation(
                        grabMiddleChainRed.getHeading(),
                        scoreMiddleChainRed.getHeading())
                .build();

        gateIntakeChainRed = follower.pathBuilder()
                .addPath(new BezierLine(scoreMiddleChainRed, gateIntakeRed))
                .setLinearHeadingInterpolation(
                        scoreMiddleChainRed.getHeading(),
                        gateIntakeRed.getHeading())
                .build();

        scoreGateChainRed = follower.pathBuilder()
                .addPath(new BezierLine(gateIntakeRed, scoreGateRed))
                .setLinearHeadingInterpolation(
                        gateIntakeRed.getHeading(),
                        scoreGateRed.getHeading())
                .build();

        toTopChainRedPath = follower.pathBuilder()
                .addPath(new BezierCurve(scoreGateRed, controlToTopRed, toTopChainRed))
                .setLinearHeadingInterpolation(
                        scoreGateRed.getHeading(),
                        toTopChainRed.getHeading())
                .build();

        grabTopChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(toTopChainRed, grabTopChainRed))
                .setTangentHeadingInterpolation()
                .build();

        scoreTopChainRedPath = follower.pathBuilder()
                .addPath(new BezierCurve(grabTopChainRed, controlScoreTopRed, scoreTopChainRed))
                .setLinearHeadingInterpolation(
                        grabTopChainRed.getHeading(),
                        scoreTopChainRed.getHeading())
                .build();

        toBottomChainRedPath = follower.pathBuilder()
                .addPath(new BezierCurve(scoreTopChainRed, controlToBottomRed, toBottomChainRed))
                .setLinearHeadingInterpolation(
                        scoreTopChainRed.getHeading(),
                        toBottomChainRed.getHeading())
                .build();

        grabBottomChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(toBottomChainRed, grabBottomChainRed))
                .setTangentHeadingInterpolation()
                .build();

        scoreBottomChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(grabBottomChainRed, scoreBottomChainRed))
                .setLinearHeadingInterpolation(
                        grabBottomChainRed.getHeading(),
                        scoreBottomChainRed.getHeading())
                .build();

        leaveRed = follower.pathBuilder()
                .addPath(new BezierLine(scoreBottomChainRed, leavePathRed))
                .setTangentHeadingInterpolation()
                .build();
    }

    private void autonomousPathUpdate() {
        if (!pathStarted) {
            switch (pathState) {
                case 0: follower.followPath(scorePreload); break;
                case 1: follower.followPath(toFirstChain); break;
                case 2: follower.followPath(grabMiddleChain); break;
                case 3: follower.followPath(scoreMiddleChain); break;
                case 4: follower.followPath(gateIntakeChain); break;
                case 5: follower.followPath(scoreGateChain); break;
                case 6: follower.followPath(toTopChain); break;
                case 7: follower.followPath(grabTopChain); break;
                case 8: follower.followPath(scoreTopChain); break;
                case 9: follower.followPath(toBottomChain); break;
                case 10: follower.followPath(grabBottomChain); break;
                case 11: follower.followPath(scoreBottomChain); break;
                case 12: follower.followPath(leave); break;
            }
            pathStarted = true;
        }

        if (pathStarted && !follower.isBusy()) {
            pathState++;
            pathStarted = false;
            pathTimer.resetTimer();
        }
    }

    private void nextPath() {
        pathState++;
        pathTimer.resetTimer();
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

}
