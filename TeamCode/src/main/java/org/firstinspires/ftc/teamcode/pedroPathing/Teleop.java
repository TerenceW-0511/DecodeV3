package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import java.util.Arrays;

@Config
@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode {
    public Follower follower;
    public Hardware hardware;
    private Methods intakePID;
    private Methods transferPID;
    private Methods flywheelPID;
    private Methods methods;

    private Timer timer;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private boolean speedFirstLoop = true;
    private boolean waitingForLimiter = false;
    private double limiterOpenTime = 0;
 //TODO: add flywheel idle, turret no auto aim

    double targetHeading = Math.toRadians(180); // Radians
    PIDFController controller;
    boolean headingLock = false;

    Values.Team lastTeam;




    public void init(){
        hardware = new Hardware(hardwareMap);

        intakePID = new Methods();
        transferPID = new Methods();
        flywheelPID = new Methods();
        methods = new Methods();
        timer = new Timer();
        Values.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(Values.autonFollowerX,Values.autonFollowerY,Values.autonHeading));
        follower.update();
        controller = new com.pedropathing.control.PIDFController(follower.constants.coefficientsHeadingPIDF);
    }
    @Override
    public void init_loop(){
//        Values.turretOverride -= gamepad1.left_trigger/300;
//        Values.turretOverride += gamepad1.right_trigger/300;
//        hardware.turret1.setPosition(Values.turretPos+Values.turretOverride);
//        hardware.turret2.setPosition(Values.turretPos+Values.turretOverride);

    }
    @Override
    public void start(){
        hardware.ll.start();
        follower.startTeleopDrive();
        timer.resetTimer();
        lastTeam = Values.team;
        hardware.kicker.setPosition(Values.KICKER_DOWN);
        hardware.ll.pipelineSwitch(Values.team == Values.Team.RED ? 1 : 2);
    }


    @Override
    public void loop() {
        follower.update();
        controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        controller.updateError(getHeadingError());

        if (headingLock){
            follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                    controller.run()
            );
        }else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
        }
        //UPDATE VARS
        Pose pose = follower.getPose();
        double dist = methods.getDist(pose);
        double flywheelVel1 = hardware.flywheel1.getVelocity();
        double flywheelVel2 = hardware.flywheel2.getVelocity();

        //MODE SWITCHING
        if (gamepad1.leftBumperWasPressed() && Values.mode== Values.Modes.SHOOTING){
            Values.init = true;
            Values.mode=Values.Modes.INTAKING;
            hardware.kicker.setPosition(Values.KICKER_UP);
            timer.resetTimer();

        }
        if (gamepad1.rightBumperWasPressed() && Values.mode == Values.Modes. INTAKING){
            Values.init=true;
            Values.mode = Values.Modes.SHOOTING;
            timer.resetTimer();
        }




        if (gamepad1.bWasPressed()){
            methods.manualRelocalize(follower);
        }
        if (gamepad1.xWasPressed()){
            Values.tx=0;
        }

        if (Values.team != lastTeam) {
            hardware.ll.pipelineSwitch(Values.team == Values.Team.RED ? 1 : 2);
            lastTeam = Values.team;
        }

        if (gamepad1.leftStickButtonWasPressed()){
            hardware.ll.start();
            if (Values.team==Values.Team.RED){
                Values.team = Values.Team.BLUE;
            }else{
                Values.team=Values.Team.RED;
                }
        }

        if (gamepad1.rightStickButtonWasPressed()){
            headingLock = !headingLock;
            controller.reset();
        }
        if (gamepad1.dpadUpWasPressed()){
            Values.flywheel_Values.flywheelTarget+=30;
        }else if (gamepad1.dpadDownWasPressed()){
            Values.flywheel_Values.flywheelTarget-=30;
        }

        if (gamepad1.aWasPressed()){
            Values.hoodPos += 0.02;
        }else if (gamepad1.yWasPressed()){
            Values.hoodPos -=0.02;
        }

//        Values.hoodPos = methods.hoodControl(dist,hardware.flywheel1,hardware.flywheel2);
//        Values.flywheel_Values.flywheelTarget=methods.flywheelControl(follower,hardware.hood1);
        hardware.hood1.setPosition(Values.hoodPos);

//        Values.flywheel_Values.flywheelTarget=methods.flywheelControl(follower,hardware.hood1);
//        hardware.hood1.setPosition(methods.hoodControl(dist, hardware.flywheel1, hardware.flywheel2));





        switch(Values.mode) {
            case INTAKING:
                flywheelPID.flywheelFFTele(hardware.flywheel1,hardware.flywheel2,Values.flywheel_Values.flywheelIdle);
                if (Values.init && timer.getElapsedTimeSeconds()>0.4){
                    Values.init=false;
                    hardware.kicker.setPosition(Values.KICKER_DOWN);
                }
                hardware.led.setPosition(0.333); //orange=
                if (timer.getElapsedTimeSeconds()>0.1) {
                    hardware.limiter.setPosition(Values.LIMITER_CLOSE);
                }
                if (gamepad1.left_bumper) {
//                    Values.intake_Values.intakeTarget = Values.intake_Values.intakeIntaking;
                    hardware.intake.setPower(1);
                    transferPID.velocity_PID(hardware.transfer,Values.transfer_Values.transferIntake,"transfer");
//                    Values.transfer_Values.transferTarget = Values.transfer_Values.transferIntake;
                } else {

                    hardware.intake.setPower(0);
//                    Values.intake_Values.intakeTarget = 2*Values.intake_Values.intakeHold;
                    //Values.transfer_Values.transferTarget = 0;
                    hardware.transfer.setPower(0);
                }
                break;
            case SHOOTING:
//                if (Values.init){
//                   Values.tx=methods.limelightCorrection(hardware.ll,dist);
//                    Values.turretPos = methods.AutoAim(follower.getPose(),hardware.ll);
//                    Values.init=false;
//                }
//                if (gamepad1.rightBumperWasPressed()){
//                methods.limelightCorrection(hardware.ll,dist);
//                Values.turretPos = methods.AutoAim(follower.getPose(),hardware.ll);
                flywheelPID.flywheelFFTele(hardware.flywheel1,hardware.flywheel2,Values.flywheel_Values.flywheelTarget);
                double rpmError = Math.abs((flywheelVel1+flywheelVel2)/2 - Values.flywheel_Values.flywheelTarget);
                hardware.limiter.setPosition(Values.LIMITER_OPEN);
                hardware.kicker.setPosition(Values.KICKER_DOWN);
                if (rpmError > 80){
                    hardware.led.setPosition(0.722); //purple
                }else{
                    hardware.led.setPosition(0.444); //green
                }

                if (gamepad1.right_bumper && !Values.turretDeadSpot) {

                    boolean atSpeed = rpmError < 70;

                    if (headingLock) {
                        atSpeed = atSpeed && Math.abs(follower.getAngularVelocity()) <0.1;
                    }

                    if (atSpeed && timer.getElapsedTimeSeconds() > 0.1) {
                        hardware.intake.setPower(1);
                        hardware.transfer.setPower(1);
                    }

                } else {
                    hardware.transfer.setPower(0);
                    hardware.intake.setPower(0);
                }






//
//                if (speedFirstLoop){
//                    if (timer.getElapsedTimeSeconds()>2 && timer.getElapsedTimeSeconds()>2.5){
//                        hardware.kicker.setPosition(Values.KICKER_UP);
//                    }else{
//                        hardware.kicker.setPosition(Values.KICKER_DOWN);
//                    }
//                }else if(dist>120){
//                    if (timer.getElapsedTimeSeconds()>1.6){
//                        hardware.kicker.setPosition(Values.KICKER_UP);
//                    }else{
//                        hardware.kicker.setPosition(Values.KICKER_DOWN);
//                    }
//                }else{
//                    if (timer.getElapsedTimeSeconds()>1.2){
//                        hardware.kicker.setPosition(Values.KICKER_UP);
//                    }else{
//                        hardware.kicker.setPosition(Values.KICKER_DOWN);
//                    }
//                }


                break;
        }
//        Values.turretPos-= gamepad1.right_trigger/30;
//        Values.turretPos+= gamepad1.left_trigger/30;
//        Values.turretPos=Math.min(Math.max(0,Values.turretPos),1);
//        if (timer.getElapsedTimeSeconds()>0.5) {
//            Values.turretPos = methods.AutoAim(follower.getPose(),hardware.ll);
//            double llOutput = methods.limelightCorrection(hardware.ll,dist);
//            telemetry.addData("tx",llOutput);
//            if (follower.getAngularVelocity()<0.2 && follower.getVelocity().getMagnitude()<0.5 && Math.abs(Values.lastTurret-Values.turretPos)<0.02) {
//                hardware.ll.start();
//                double[] llOutput = methods.limelightCorrection(hardware.ll, dist);
//                if (llOutput.length == 1) {
//                    telemetry.addData("tx", "invalid");
//                } else {
//                    telemetry.addData("tx,target,output", Arrays.toString(llOutput));
//                    TelemetryPacket packet = new TelemetryPacket();
//                    packet.put("tx", llOutput[0]);
//                    packet.put("target", llOutput[1]);
//                    dashboard.sendTelemetryPacket(packet);
//                }
//            }else{
//                hardware.ll.pause();
//            }

//        }

        methods.limelightCorrection(hardware.ll, dist);
        if (hardware.ll.getTimeSinceLastUpdate()>0.5){
            hardware.ll.reloadPipeline();
            Values.tx=0;
        }
        double turretEncoder = -hardware.intake.getCurrentPosition();
        if (headingLock){
            Values.turretPos = methods.turretPID(turretEncoder,0);
        }else {
            double targetTurret = methods.AutoAim(pose, hardware.ll);


            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Turret Target", targetTurret);
            packet.put("Turret Current", -hardware.intake.getCurrentPosition());
            packet.put("target vel", Values.flywheel_Values.flywheelTarget);
            packet.put("curr vel", flywheelVel1);
            dashboard.sendTelemetryPacket(packet);

            Values.turretOverride += gamepad1.left_trigger * 500;
            Values.turretOverride -= gamepad1.right_trigger * 500;
            Values.turretOverride = Math.min(5000, Math.max(-5000, Values.turretOverride));
            Values.turretPos = methods.turretPID(turretEncoder, targetTurret + Values.turretOverride);
            hardware.turret1.setPosition(Values.turretPos);
            hardware.turret2.setPosition(Values.turretPos);
            telemetry.addData("turret pos",targetTurret+Values.turretOverride);
        }

        telemetry.addData("heading lock",headingLock);






//        Values.lastTurret=Values.turretPos;

        telemetry.addData("mode",Values.mode);
        telemetry.addData("ll conencted",hardware.ll.getStatus());
        telemetry.addData("ll running",hardware.ll.isRunning());
        telemetry.addData("tx",Values.tx);

        telemetry.addData("team",Values.team);
        telemetry.addData("pose",pose);
        telemetry.addData("heading",Math.toDegrees(follower.getHeading())*1.0006);
        telemetry.addData("dist",dist);
        telemetry.addData("flywheel target",Values.flywheel_Values.flywheelTarget);
        telemetry.addData("flywheel rpm", String.format("1: %f,2: %f",flywheelVel1,flywheelVel2));
        telemetry.addData("flywheel power",String.format("1: %f,2: %f",hardware.flywheel1.getPower(),hardware.flywheel2.getPower()));
        telemetry.addData("hood pos",Values.hoodPos);
        telemetry.addData("turret position",turretEncoder);
        telemetry.addData("turret override",Values.turretOverride);

        telemetry.update();

    }
    public double getHeadingError() {
        double dx, dy, alpha;
        Pose botPose = follower.getPose();
        if (Values.team == Values.Team.BLUE) {
            dx = botPose.getX() - Values.blueGoal.getX();
            dy = Values.blueGoal.getY() - botPose.getY();
            alpha = 180
                    - Math.toDegrees(botPose.getHeading())
                    - Math.toDegrees(Math.atan2(dy, dx));
        } else {
            dx = botPose.getX() - Values.redGoal.getX();
            dy = Values.redGoal.getY() - botPose.getY();
            alpha = 180
                    - Math.toDegrees(botPose.getHeading())
                    - Math.toDegrees(Math.atan2(dy, dx));
        }
        return MathFunctions.getTurnDirection(follower.getPose().getHeading(), alpha) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), alpha);
    }


}
