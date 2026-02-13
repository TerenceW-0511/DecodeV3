package org.firstinspires.ftc.teamcode.pedroPathing;
//we are skibidi, we are rizzlers
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auton 12 Spike", group = "Auto")
public class AutonSpike12 extends OpMode {

    private Hardware robot;
    private Follower follower;
    private Methods intakePID,transferPID,flywheelPID,methods;
    private boolean pathStarted = false;
    private static final double CALLBACK_TIME = 10.0;

    private boolean emergencyTriggered = false;
    private boolean case15Completed = false;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private static final double STABLE_TIME = 0.10;



    // PATH CHAINS
// BLUE
    public PathChain scorePreloadBlue;
    public PathChain toFirstChainBlue;
    public PathChain grabMiddleChainBlue;
    public PathChain openGatePathBlue;
    public PathChain toMiddleChainBlue,scoreMiddleChainBluePath;
    public PathChain grabTopChainBluePath;
    public PathChain scoreTopChainBluePath;
    public PathChain toBottomChainBluePath;
    public PathChain grabBottomChainBluePath;
    public PathChain scoreBottomChainBluePath;

    // RED
    public PathChain scorePreloadRedChain;
    public PathChain toFirstChainRed;
    public PathChain toMiddleChainRed,grabMiddleChainRedPath;
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
    public static Pose startingPoseBlue = new Pose(33.3,136.7,Math.toRadians(180));//0
    public static Pose scorePreloadPoseBlue = new Pose(55.4,84,Math.toRadians(180)); //1
    public static Pose toFirstBlue = new Pose(48,60,Math.toRadians(180)); //5
    public static Pose controlToFirstBlue = new Pose(56.1,66.5);
    public static Pose grabMiddleBlue = new Pose(10.5,60,Math.toRadians(180)); //6
    public static Pose openGateBlue = new Pose(15.9,74,Math.toRadians(180)); //3
    public static Pose controlOpenGateBlue = new Pose(27.67,77.2);
    public static Pose scoreMiddleChainBlue = new Pose(59.4,88,Math.toRadians(180)); //7
    public static Pose controlScoreMiddleBlue = new Pose(41.2,66.9);
    public static Pose grabTopChainBlue = new Pose(18,88,Math.toRadians(180)); //2
    public static Pose scoreTopChainBlue = new Pose(58,74,Math.toRadians(180)); //4
    public static Pose toBottomChainBlue = new Pose(43.75,36.2,Math.toRadians(180)); //8
    public static Pose controlToBottomBlue = new Pose(53.6,35.7);
    public static Pose grabBottomChainBlue = new Pose(10.4,36.2,Math.toRadians(180));
    public static Pose scoreBottomChainBlue = new Pose(58,110,Math.toRadians(180));

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
        isRed = false;
        pathTimer = new Timer();
        actionTimer = new Timer();

        robot = new Hardware(hardwareMap);
        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        robot.limiter.setPosition(Values.LIMITER_CLOSE);
        TelemetryPacket packet = new TelemetryPacket();

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
        grabTopChain     = isRed ? grabTopChainRedPath      : grabTopChainBluePath;
        openGateChain    = isRed ? openGatePathRed              : openGatePathBlue;

        scoreTopChain    = isRed ? scoreTopChainRedPath     : scoreTopChainBluePath;
        toMiddleChain    = isRed ? toMiddleChainRed          : toMiddleChainBlue;
        grabMiddleChain  = isRed ? grabMiddleChainRedPath   : grabMiddleChainBlue;

        scoreMiddleChain = isRed ? scoreMiddleChainRedPath  : scoreMiddleChainBluePath;



        toBottomChain    = isRed ? toBottomChainRedPath     : toBottomChainBluePath;
        grabBottomChain  = isRed ? grabBottomChainRedPath   : grabBottomChainBluePath;
        scoreBottomChain = isRed ? scoreBottomChainRedPath  : scoreBottomChainBluePath;



        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        Pose pose = follower.getPose();
        double dist = methods.getDist(pose);
        double flywheelVel1 = robot.flywheel1.getVelocity();
        double flywheelVel2 = robot.flywheel2.getVelocity();
        Values.flywheel_Values.flywheelTarget=methods.flywheelControl(follower,robot.hood1);
        flywheelPID.flywheelFFTele(robot.flywheel1, robot.flywheel2,Values.flywheel_Values.flywheelTarget);
        double rpmError = Math.abs((flywheelVel1+flywheelVel2)/2 - Values.flywheel_Values.flywheelTarget);

//        intakePID.velocity_PID(robot.intake,Values.intake_Values.intakeTarget,"intake");
//        transferPID.velocity_PID(robot.transfer,Values.transfer_Values.transferTarget,"transfer");
        Values.hoodPos = methods.hoodControl(dist,robot.flywheel1,robot.flywheel2);
        robot.hood1.setPosition(Values.hoodPos);
        methods.limelightCorrection(robot.ll,dist);
        double targetTurret = methods.AutoAim(follower.getPose(),robot.ll);
        double turretEncoder = -robot.intake.getCurrentPosition();
        Values.turretPos = methods.turretPID(turretEncoder, targetTurret+Values.turretOverride);
        robot.turret1.setPosition(Values.turretPos);
        robot.turret2.setPosition(Values.turretPos);


        Values.autonFollowerX = follower.getPose().getX();
        Values.autonFollowerY = follower.getPose().getY();
        Values.autonHeading = Math.toDegrees(follower.getHeading());
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Vel", Values.flywheel_Values.flywheelTarget);
        packet.put("Curr Vel", (flywheelVel1+flywheelVel2)/2);
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Path State", pathState);
        telemetry.addData("drive error",follower.getDriveError());
        telemetry.addData("heading error",follower.getHeadingError());
        telemetry.addData("angular velocity",follower.getAngularVelocity());
        telemetry.addData("velocity",follower.getVelocity().getMagnitude());
        telemetry.addData("flywheel target",Values.flywheel_Values.flywheelTarget);
        telemetry.addData("flywheel rpm", String.format("1: %f,2: %f",flywheelVel1,flywheelVel2));
        telemetry.update();
    }

    public void intake(){
        follower.setMaxPower(0.7);
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
//        Values.intake_Values.intakeTarget=Values.intake_Values.intakeIntaking;
//        Values.transfer_Values.transferTarget=Values.transfer_Values.transferIntake;
        robot.intake.setPower(1);
        transferPID.velocity_PID(robot.transfer,Values.transfer_Values.transferIntake,"transfer");
    }
    public void move(){
        follower.setMaxPower(1);
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
//        Values.intake_Values.intakeTarget=Values.intake_Values.intakeHold*2;
        robot.intake.setPower(1);
        robot.transfer.setPower(0);
//        Values.transfer_Values.transferTarget=0;
    }
    public void moveNoIntake(){
        follower.setMaxPower(1);
        robot.limiter.setPosition(Values.LIMITER_CLOSE);
//        Values.intake_Values.intakeTarget=0;
//        Values.transfer_Values.transferTarget=0;
        robot.transfer.setPower(0);
        robot.intake.setPower(0);
    }
    public boolean shoot(){

        robot.limiter.setPosition(Values.LIMITER_OPEN);

        double flywheelVel1 = robot.flywheel1.getVelocity();
        double flywheelVel2 = robot.flywheel2.getVelocity();
        double avgFlywheel = (flywheelVel1 + flywheelVel2) / 2.0;

        double rpmError = Math.abs(avgFlywheel - Values.flywheel_Values.flywheelTarget);

        double turretEncoder = -robot.intake.getCurrentPosition();
        double targetTurret = methods.AutoAim(follower.getPose(), robot.ll) + Values.turretOverride;
        double turretError = Math.abs(targetTurret - turretEncoder);

        boolean turretStable = turretError < 200;

        boolean fullyStable = turretStable;

        if (!fullyStable) {
            actionTimer.resetTimer();
            return false;
        }

        if (actionTimer.getElapsedTimeSeconds() < STABLE_TIME) {
            return false;
        }

        // ---- FIRE ----
        robot.intake.setPower(1);
        robot.transfer.setPower(1);

        return pathTimer.getElapsedTimeSeconds() > 1.5;
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

        grabTopChainBluePath = follower.pathBuilder()
                .addPath(new BezierLine(
                        scorePreloadPoseBlue,
                        grabTopChainBlue))
                .setLinearHeadingInterpolation(
                        scorePreloadPoseBlue.getHeading(),
                        grabTopChainBlue.getHeading())

                .setHeadingConstraint(0.3)
                .build();

        openGatePathBlue = follower.pathBuilder()
                .addPath(new BezierCurve(
                        grabTopChainBlue,
                        controlOpenGateBlue,
                        openGateBlue))
                .setLinearHeadingInterpolation(
                        grabTopChainBlue.getHeading(),
                        openGateBlue.getHeading())

                .setHeadingConstraint(0.3)
                .build();

        scoreTopChainBluePath = follower.pathBuilder() //4
                .addPath(new BezierLine(
                        openGateBlue,
                        scoreTopChainBlue))
                .setLinearHeadingInterpolation(
                        openGateBlue.getHeading(),
                        scoreTopChainBlue.getHeading())

                .setHeadingConstraint(0.3)
                .build();
        toMiddleChainBlue = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scoreTopChainBlue,
                        controlToFirstBlue,
                        toFirstBlue))
                .setLinearHeadingInterpolation(
                        scoreTopChainBlue.getHeading(),
                        toFirstBlue.getHeading())

                .setHeadingConstraint(0.3)
                .build();

        grabMiddleChainBlue = follower.pathBuilder()
                .addPath(new BezierLine(toFirstBlue, grabMiddleBlue))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();


        scoreMiddleChainBluePath = follower.pathBuilder()
                .addPath(new BezierCurve(grabMiddleBlue, controlScoreMiddleBlue,scoreMiddleChainBlue))
                .setLinearHeadingInterpolation(
                        grabMiddleBlue.getHeading(),
                        scoreMiddleChainBlue.getHeading())
                .setHeadingConstraint(0.3)
                .build();



        toBottomChainBluePath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scoreMiddleChainBlue,
                        controlToBottomBlue,
                        toBottomChainBlue))
                .setLinearHeadingInterpolation(
                        scoreMiddleChainBlue.getHeading(),
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

        scorePreloadRedChain= follower.pathBuilder()
                .addPath(new BezierLine(startingPoseRed, scorePreloadPoseRed))
                .setLinearHeadingInterpolation(
                        startingPoseRed.getHeading(),
                        scorePreloadPoseRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();

        grabTopChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        scorePreloadPoseRed,
                        grabTopChainRed))
                .setLinearHeadingInterpolation(
                        scorePreloadPoseRed.getHeading(),
                        grabTopChainRed.getHeading())

                .setHeadingConstraint(0.3)
                .build();

        openGatePathRed = follower.pathBuilder()
                .addPath(new BezierCurve(
                        grabTopChainRed,
                        controlOpenGateRed,
                        openGateRed))
                .setLinearHeadingInterpolation(
                        grabTopChainRed.getHeading(),
                        openGateRed.getHeading())

                .setHeadingConstraint(0.3)
                .build();

        scoreTopChainRedPath = follower.pathBuilder() //4
                .addPath(new BezierLine(
                        openGateRed,
                        scoreTopChainRed))
                .setLinearHeadingInterpolation(
                        openGateRed.getHeading(),
                        scoreTopChainRed.getHeading())

                .setHeadingConstraint(0.3)
                .build();
        toMiddleChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scoreTopChainRed,
                        controlToFirstRed,
                        toFirstRed))
                .setLinearHeadingInterpolation(
                        scoreTopChainRed.getHeading(),
                        toFirstRed.getHeading())

                .setHeadingConstraint(0.3)
                .build();

        grabMiddleChainRedPath = follower.pathBuilder()
                .addPath(new BezierLine(toFirstRed, grabMiddleRed))
                .setTangentHeadingInterpolation()
                .setHeadingConstraint(0.3)
                .build();


        scoreMiddleChainRedPath = follower.pathBuilder()
                .addPath(new BezierCurve(grabMiddleRed, controlScoreMiddleRed,scoreMiddleChainRed))
                .setLinearHeadingInterpolation(
                        grabMiddleRed.getHeading(),
                        scoreMiddleChainRed.getHeading())
                .setHeadingConstraint(0.3)
                .build();



        toBottomChainRedPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scoreMiddleChainRed,
                        controlToBottomRed,
                        toBottomChainRed))
                .setLinearHeadingInterpolation(
                        scoreMiddleChainRed.getHeading(),
                        toBottomChainRed.getHeading())
                .setNoDeceleration()
                .setHeadingConstraint(0.3)
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
            case 17:

                if (Math.abs(Values.flywheel_Values.flywheelTarget-robot.flywheel1.getVelocity())<80 && follower.getAngularVelocity()<.1 && follower.getVelocity().getMagnitude()<.1) {
                    nextPath();
                }
                break;
            case 3:
            case 8:
            case 13:
            case 18:

                if (shoot()){
                    nextPath();
                }
                break;

            // grab middle & open gate
            case 4:
                intake();
                runPath(grabTopChain);
                break;

            case 5:
                if (follower.getPathCompletion()>0.5) {
                    moveNoIntake();
                }else{
                    intake();
                }
                runPath(openGateChain);
                break;
            case 6:
                move();
                runPath(scoreTopChain);
                break;
            // grab top
            case 9:
                move();
                runPath(toMiddleChain);
                break;
            case 10:
                intake();
                runPath(grabMiddleChain);
                break;
            //grab bottom
            case 12:
                move();
                runPath(scoreMiddleChain);
                break;
            case 14:
                move();
                runPath(toBottomChain);
                break;
            case 15:
                intake();
                runPath(grabBottomChain);
                break;
            case 16:
                move();
                runPath(scoreBottomChain);
                break;
            case 19:
                Values.turretPos = methods.turretPID(Values.autonTurret, 0);





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
        if (pathState==5) {
            follower.setMaxPower(0.7);
            if (pathTimer.getElapsedTimeSeconds() > 4) {
                nextPath();
            }
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
