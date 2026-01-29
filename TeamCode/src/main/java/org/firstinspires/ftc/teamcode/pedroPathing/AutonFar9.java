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

@Autonomous(name = "Auton Far 9", group = "Auto")
public class AutonFar9 extends OpMode {

    private Hardware robot;
    private Follower follower;
    private Methods intakePID,transferPID,flywheelPID,methods;
    private boolean pathStarted = false;



    // PATH CHAINS
// BLUE
    public PathChain scorePreloadBlue;
    public PathChain grabBottomChainBlue;
    public PathChain scoreBottomChainBlue;
    public PathChain grabLoadedChainBlue;
    public PathChain scoreLoadedChainBlue;
    public PathChain leaveBlue;

    // RED
    public PathChain scorePreloadRed;
    public PathChain grabBottomChainRed;
    public PathChain scoreBottomChainRed;
    public PathChain grabLoadedChainRed;
    public PathChain scoreLoadedChainRed;
    public PathChain leaveRed;
    // FINAL SELECTED CHAINS
    private PathChain scorePreload;
    private PathChain grabBottomChain, scoreBottomChain;
    private PathChain grabLoadedChain, scoreLoadedChain;
    private PathChain leave;


    private int pathState, actionState;
    private Timer pathTimer, actionTimer;

    public static Pose startingPose;

    // BLUE POSES+
    public static Pose startingPoseBlue = new Pose(55.2,8.8,Math.toRadians(270));
    public static Pose scorePreloadPoseBlue = new Pose(56,17.1,Math.toRadians(180));
    public static Pose toBottomBlue = new Pose(12.9,35.2,Math.toRadians(180));
    public static Pose controlToBottomBlue = new Pose(59.3,39.2);
    public static Pose scoreBottomBlue = new Pose(58.9,20.1,Math.toRadians(180));
    public static Pose toLoadedBlue = new Pose(13.4,10.2,Math.toRadians(180));
    public static Pose controlLoadedBlue = new Pose(57.1,8.9);
    public static Pose scoreLoadedBlue = new Pose(50.5,10.2,Math.toRadians(180));
    public static Pose leavePoseBlue = new Pose(35.2,10.2,Math.toRadians(180));



    // RED POSES
    public static Pose startingPoseRed = mirrorPose(startingPoseBlue);
    public static Pose scorePreloadPoseRed = mirrorPose(scorePreloadPoseBlue);
    public static Pose toBottomRed = mirrorPose(toBottomBlue);
    public static Pose controlToBottomRed = mirrorPoint(controlToBottomBlue);
    public static Pose scoreBottomRed = mirrorPose(scoreBottomBlue);
    public static Pose toLoadedRed = mirrorPose(toLoadedBlue);
    public static Pose controlLoadedRed = mirrorPoint(controlLoadedBlue);
    public static Pose scoreLoadedRed = mirrorPose(scoreLoadedBlue);
    public static Pose leavePoseRed = mirrorPose(leavePoseBlue);


    private boolean isRed = true;





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
            Values.team = Values.Team.RED;
            startingPose = startingPoseRed;
        }
        Values.turretOverride -= gamepad1.left_trigger/300;
        Values.turretOverride += gamepad1.right_trigger/300;
        robot.turret1.setPosition(Values.turretPos+Values.turretOverride);
        robot.turret2.setPosition(Values.turretPos+Values.turretOverride);
        robot.kicker.setPosition(Values.KICKER_DOWN);

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

        scorePreload     = isRed ? scorePreloadRed     : scorePreloadBlue;
        grabBottomChain  = isRed ? grabBottomChainRed  : grabBottomChainBlue;
        scoreBottomChain = isRed ? scoreBottomChainRed : scoreBottomChainBlue;
        grabLoadedChain  = isRed ? grabLoadedChainRed  : grabLoadedChainBlue;
        scoreLoadedChain = isRed ? scoreLoadedChainRed : scoreLoadedChainBlue;
        leave            = isRed ? leaveRed            : leaveBlue;


        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        Values.flywheel_Values.flywheelTarget=methods.flywheelControl(follower,robot.hood1);
        flywheelPID.flywheelFF(robot.flywheel1, robot.flywheel2,Values.flywheel_Values.flywheelTarget);
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
        robot.kicker.setPosition(Values.KICKER_DOWN);
//        Values.intake_Values.intakeTarget=Values.intake_Values.intakeIntaking;
//        Values.transfer_Values.transferTarget=Values.transfer_Values.transferIntake;
        robot.intake.setPower(1);
        robot.transfer.setPower(.7);
    }
    public void move(){
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        robot.kicker.setPosition(Values.KICKER_DOWN);
//        Values.intake_Values.intakeTarget=Values.intake_Values.intakeHold*2;
        robot.intake.setPower(0);
        robot.transfer.setPower(1);
//        Values.transfer_Values.transferTarget=0;
    }
    public void moveNoIntake(){
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        robot.kicker.setPosition(Values.KICKER_DOWN);
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

        grabBottomChainBlue = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePreloadPoseBlue,
                        controlToBottomBlue,
                        toBottomBlue))
                .setConstantHeadingInterpolation(toBottomBlue.getHeading())
                .setNoDeceleration()
                .setHeadingConstraint(0.3)
                .build();

        scoreBottomChainBlue = follower.pathBuilder()
                .addPath(new BezierLine(toBottomBlue, scoreBottomBlue))
                .setConstantHeadingInterpolation(scoreBottomBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();
        grabLoadedChainBlue = follower.pathBuilder()
                .addPath(new BezierCurve(scoreBottomBlue,controlLoadedBlue,toLoadedBlue))
                .setConstantHeadingInterpolation(toLoadedBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        scoreLoadedChainBlue = follower.pathBuilder()
                .addPath(new BezierCurve(toLoadedBlue,scoreLoadedBlue))
                .setConstantHeadingInterpolation(scoreLoadedBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();


        leaveBlue = follower.pathBuilder()
                .addPath(new BezierLine(scoreLoadedBlue, leavePoseBlue))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();

        // =======================
        // RED PATHS
        // =======================

        scorePreloadRed = follower.pathBuilder()
                .addPath(new BezierLine(startingPoseRed, scorePreloadPoseRed))
                .setLinearHeadingInterpolation(
                        startingPoseRed.getHeading(),
                        scorePreloadPoseRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        grabBottomChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePreloadPoseRed,
                        controlToBottomRed,
                        toBottomRed))
                .setConstantHeadingInterpolation(toBottomRed.getHeading())
                .setNoDeceleration()
                .setHeadingConstraint(0.3)
                .build();

        scoreBottomChainRed = follower.pathBuilder()
                .addPath(new BezierLine(toBottomRed, scoreBottomRed))
                .setConstantHeadingInterpolation(scoreBottomRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();
        grabLoadedChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(scoreBottomRed,controlLoadedRed,toLoadedRed))
                .setConstantHeadingInterpolation(toLoadedRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        scoreLoadedChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(toLoadedRed,scoreLoadedRed))
                .setConstantHeadingInterpolation(scoreLoadedRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();


        leaveRed = follower.pathBuilder()
                .addPath(new BezierLine(scoreLoadedRed, leavePoseRed))
                .setTangentHeadingInterpolation()
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
            follower.setMaxPower(0.6);
            if (pathTimer.getElapsedTimeSeconds()>2){
                nextPath();
            }
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
