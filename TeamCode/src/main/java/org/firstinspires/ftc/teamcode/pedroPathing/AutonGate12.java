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

@Autonomous(name = "Auton 12 Gate", group = "Auto")
public class AutonGate12 extends OpMode {

    private Hardware robot;
    private Follower follower;
    private Methods intakePID,transferPID,flywheelPID,methods;
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

    // BLUE POSES+
    public static Pose startingPoseBlue = new Pose(18,120,Math.toRadians(144));
    public static Pose scorePreloadPoseBlue = new Pose(55.2,89,Math.toRadians(144));
    public static Pose toFirstBlue = new Pose(48,60,Math.toRadians(180));
    public static Pose controlToFirstBlue = new Pose(56.1,66.5);
    public static Pose grabMiddleBlue = new Pose(13,60,Math.toRadians(180));
    public static Pose scoreMiddleChainBlue = new Pose(53,81,Math.toRadians(180));
    public static Pose gateIntakeBlue = new Pose(12.7,60.8,Math.toRadians(140));
    public static Pose controlScoreGateBlue = new Pose(61.2,61.7);
    public static Pose scoreGateBlue = new Pose(54,84.1,Math.toRadians(140));
    public static Pose toTopChainBlue = new Pose(45.2,84.3,Math.toRadians(180));
    public static Pose controlToTopBlue = new Pose(40.6,67.4);
    public static Pose grabTopChainBlue = new Pose(17.5,84.1,Math.toRadians(180));
    public static Pose scoreTopChainBlue = new Pose(55,84.1,Math.toRadians(180));
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
    public static Pose controlScoreGateRed = mirrorPoint(controlScoreGateBlue);
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
        Values.turretOverride -= gamepad1.left_trigger/300;
        Values.turretOverride += gamepad1.right_trigger/300;
        robot.turret1.setPosition(Values.turretPos+Values.turretOverride);
        robot.turret2.setPosition(Values.turretPos+Values.turretOverride);

        telemetry.addData("Team", Values.team);
        telemetry.addData("Starting Pose", startingPose);
        telemetry.addData("target",follower.getCurrentPath());
        telemetry.update();
    }

    @Override
    public void start() {

        isRed = (Values.team == Values.Team.RED);

        buildPaths();

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
        Values.flywheel_Values.flywheelTarget=methods.flywheelControl(follower,robot.hood1);
//        flywheelPID.flywheelFF(robot.flywheel1, robot.flywheel2,Values.flywheel_Values.flywheelTarget);
        intakePID.velocity_PID(robot.intake,Values.intake_Values.intakeTarget,"intake");
        transferPID.velocity_PID(robot.transfer,Values.transfer_Values.transferTarget,"transfer");
        robot.hood1.setPosition(methods.hoodControl(methods.getDist(follower.getPose()),robot.flywheel1,robot.flywheel2));
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
//        Values.intake_Values.intakeTarget=Values.intake_Values.intakeIntaking;
//        Values.transfer_Values.transferTarget=Values.transfer_Values.transferIntake;
        robot.intake.setPower(1);
        robot.transfer.setPower(.7);
    }
    public void move(){
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
//        Values.intake_Values.intakeTarget=Values.intake_Values.intakeHold*2;
        robot.intake.setPower(0);
        robot.transfer.setPower(1);
//        Values.transfer_Values.transferTarget=0;
    }
    public void moveNoIntake(){
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
//        Values.intake_Values.intakeTarget=0;
//        Values.transfer_Values.transferTarget=0;
        robot.transfer.setPower(0);
        robot.intake.setPower(0);
    }
    public boolean shoot(){
        robot.limiter.setPosition(Values.LIMITER_OPEN);
//        Values.intake_Values.intakeTarget=Values.intake_Values.intakeIntaking;
//        Values.transfer_Values.transferTarget=Values.transfer_Values.transferUp;
        robot.intake.setPower(1);
        robot.transfer.setPower(1);
        if (pathTimer.getElapsedTimeSeconds()>1.1){
            return pathTimer.getElapsedTimeSeconds() > 1.4;
        }
        return false;
    }


    private void buildPaths() {

        // BLUE
        scorePreloadBlue = follower.pathBuilder()
                .addPath(new BezierLine(startingPoseBlue, scorePreloadPoseBlue))
                .setLinearHeadingInterpolation(
                        startingPoseBlue.getHeading(),
                        scorePreloadPoseBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        toFirstChainBlue = follower.pathBuilder()
                .addPath(new BezierCurve(scorePreloadPoseBlue, controlToFirstBlue, toFirstBlue))
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

        scoreMiddleChainBluePath = follower.pathBuilder()
                .addPath(new BezierLine(grabMiddleBlue, scoreMiddleChainBlue))
                .setLinearHeadingInterpolation(
                        grabMiddleBlue.getHeading(),
                        scoreMiddleChainBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        gateIntakeChainBlue = follower.pathBuilder()
                .addPath(new BezierLine(scoreMiddleChainBlue, gateIntakeBlue))
                .setLinearHeadingInterpolation(
                        scoreMiddleChainBlue.getHeading(),
                        gateIntakeBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        scoreGateChainBlue = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntakeBlue, controlScoreGateBlue, scoreGateBlue))
                .setLinearHeadingInterpolation(
                        gateIntakeBlue.getHeading(),
                        scoreGateBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        toTopChainBluePath = follower.pathBuilder()
                .addPath(new BezierCurve(scoreGateBlue, controlToTopBlue, toTopChainBlue))
                .setLinearHeadingInterpolation(
                        scoreGateBlue.getHeading(),
                        toTopChainBlue.getHeading())
                .setNoDeceleration()
                .setHeadingConstraint(0.3)
                .build();

        grabTopChainBluePath = follower.pathBuilder()
                .addPath(new BezierLine(toTopChainBlue, grabTopChainBlue))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();

        scoreTopChainBluePath = follower.pathBuilder()
                .addPath(new BezierCurve(grabTopChainBlue, scoreTopChainBlue))
                .setLinearHeadingInterpolation(
                        grabTopChainBlue.getHeading(),
                        scoreTopChainBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        toBottomChainBluePath = follower.pathBuilder()
                .addPath(new BezierCurve(scoreTopChainBlue, controlToBottomBlue, toBottomChainBlue))
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

        leaveBlue = follower.pathBuilder()
                .addPath(new BezierLine(scoreBottomChainBlue, leavePathBlue))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();

        // RED
        scorePreloadRedChain = follower.pathBuilder()
                .addPath(new BezierLine(startingPoseRed, scorePreloadPoseRed))
                .setLinearHeadingInterpolation(
                        startingPoseRed.getHeading(),
                        scorePreloadPoseRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        toFirstChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(scorePreloadPoseRed, controlToFirstRed, toFirstRed))
                .setLinearHeadingInterpolation(
                        scorePreloadPoseRed.getHeading(),
                        toFirstRed.getHeading())
                .setHeadingConstraint(0.3)
                .setNoDeceleration()
                .build();

        grabMiddleChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(toFirstRed, grabMiddleChainRed))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();

        scoreMiddleChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(grabMiddleChainRed, scoreMiddleChainRed))
                .setLinearHeadingInterpolation(
                        grabMiddleChainRed.getHeading(),
                        scoreMiddleChainRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        gateIntakeChainRed = follower.pathBuilder()
                .addPath(new BezierLine(scoreMiddleChainRed, gateIntakeRed))
                .setLinearHeadingInterpolation(
                        scoreMiddleChainRed.getHeading(),
                        gateIntakeRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        scoreGateChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntakeRed,controlScoreGateRed, scoreGateRed))
                .setLinearHeadingInterpolation(
                        gateIntakeRed.getHeading(),
                        scoreGateRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        toTopChainRedPath = follower.pathBuilder()
                .addPath(new BezierCurve(scoreGateRed, controlToTopRed, toTopChainRed))
                .setLinearHeadingInterpolation(
                        scoreGateRed.getHeading(),
                        toTopChainRed.getHeading())
                .setHeadingConstraint(0.3)
                .setNoDeceleration()
                .build();

        grabTopChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(toTopChainRed, grabTopChainRed))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();

        scoreTopChainRedPath = follower.pathBuilder()
                .addPath(new BezierCurve(grabTopChainRed, scoreTopChainRed))
                .setLinearHeadingInterpolation(
                        grabTopChainRed.getHeading(),
                        scoreTopChainRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        toBottomChainRedPath = follower.pathBuilder()
                .addPath(new BezierCurve(scoreTopChainRed, controlToBottomRed, toBottomChainRed))
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

        leaveRed = follower.pathBuilder()
                .addPath(new BezierLine(scoreBottomChainRed, leavePathRed))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            // ================= PRE-START PAUSE =================
            case 0:
                move();
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    nextPath();
                }
                break;

            // ================= SCORE PRELOAD =================
            case 1:
                move();
                runPath(scorePreload);
                break;

            case 2:
            case 6:
            case 11:
            case 15:
                if (Math.abs(Values.flywheel_Values.flywheelTarget-robot.flywheel1.getVelocity())<50 && follower.getAngularVelocity()<.5 && follower.getVelocity().getMagnitude()<1) {
                    nextPath();
                }
                break;
            case 3:
            case 7:
            case 12:
            case 16:
                if (shoot()){
                    nextPath();
                }break;

            // ================= GRAB MIDDLE =================
            case 4:
                intake();
                runPath(grabMiddleChain);
                break;

            case 5:
                move();
                runPath(scoreMiddleChain);
                break;
            case 8:
                move();
                runPath(gateIntakeChain);
                break;
            case 9:
                intake();
                if (pathTimer.getElapsedTimeSeconds()>2){
                    nextPath();
                }break;
            case 10:
                move();
                runPath(scoreGateChain);
                break;
            case 13:
                intake();
                runPath(grabTopChain);
                break;
            case 14:
                move();
                runPath(scoreTopChain);



            // ================= SAFETY =================
            default:
                move();
                break;
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
    private void runPath(PathChain path) {

        if (!pathStarted) {
            follower.followPath(path);
            pathStarted = true;
            pathTimer.resetTimer();
        }

        boolean headingGood =
                Math.abs(follower.getHeadingError()) <follower.getCurrentPath().getPathEndHeadingConstraint();
        if (follower.atParametricEnd() && headingGood || follower.isRobotStuck() || pathTimer.getElapsedTimeSeconds()>7 || !follower.isBusy()) {
            pathStarted = false;
            pathTimer.resetTimer();
            nextPath();
        }

    }


}
