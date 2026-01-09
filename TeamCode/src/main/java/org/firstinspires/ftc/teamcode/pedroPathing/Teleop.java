package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
        follower.setStartingPose(new Pose(Values.autonFollowerX,Values.autonFollowerY));

        follower.update();
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
    }


    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x/2,
                true // Robot Centric
        );
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
        if (gamepad1.rightBumperWasPressed() && Values.mode == Values.Modes.INTAKING){
            Values.init=true;
            Values.mode = Values.Modes.SHOOTING;
        }


//        if (gamepad1.yWasPressed()){
//            Values.hoodPos+= 0.03;
//        }else if (gamepad1.aWasPressed()){
//            Values.hoodPos-=0.03;
//        }

        if (gamepad1.aWasPressed()){
            methods.manualRelocalize(follower);
        }

        if (Values.team != lastTeam) {
            hardware.ll.pipelineSwitch(Values.team == Values.Team.RED ? 1 : 2);
            lastTeam = Values.team;
        }

        if (gamepad1.leftStickButtonWasPressed()){
            if (Values.team==Values.Team.RED){
                Values.team = Values.Team.BLUE;
            }else{
                Values.team=Values.Team.RED;
            }
        }



        hardware.hood1.setPosition(methods.hoodControl(dist, hardware.flywheel1, hardware.flywheel2));





        switch(Values.mode) {
            case INTAKING:
                Values.flywheel_Values.flywheelTarget=Values.flywheel_Values.flywheelIdle;
                if (Values.init && timer.getElapsedTimeSeconds()>0.4){
                    Values.init=false;
                    hardware.kicker.setPosition(Values.KICKER_DOWN);
                }
                hardware.led.setPosition(0.333); //orange
                if (timer.getElapsedTimeSeconds()>0.1) {
                    hardware.limiter.setPosition(Values.LIMITER_CLOSE);
                }
                if (gamepad1.left_bumper) {
                    Values.intake_Values.intakeTarget = Values.intake_Values.intakeIntaking;
//                    hardware.transfer.setPower(.7);
                    Values.transfer_Values.transferTarget = Values.transfer_Values.transferIntake;
                } else {
                    Values.intake_Values.intakeTarget = 2*Values.intake_Values.intakeHold;
                    Values.transfer_Values.transferTarget = 0;
//                    hardware.transfer.setPower(0);
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
                Values.flywheel_Values.flywheelTarget=methods.flywheelControl(follower,hardware.hood1);
                double rpmError = Math.abs((flywheelVel1+flywheelVel2)/2 - Values.flywheel_Values.flywheelTarget);

                hardware.kicker.setPosition(Values.KICKER_DOWN);
                if (Values.turretDeadSpot || Math.abs(Values.tx)>3){
                    hardware.led.setPosition(0.277); //red
                }else if (Math.abs(Values.tx)<2){
                    hardware.led.setPosition(0.722);
                }else if (rpmError > 80){
                    hardware.led.setPosition(0.6); //blue
                }else{
                    hardware.led.setPosition(0.444); //green
                }


                if (gamepad1.right_bumper && !Values.turretDeadSpot) {

                    if (rpmError < 40) {
                        speedFirstLoop = false;
                        hardware.limiter.setPosition(Values.LIMITER_OPEN);
                        Values.transfer_Values.transferTarget = Values.transfer_Values.transferUp;
                        Values.intake_Values.intakeTarget = Values.intake_Values.intakeShoot;
                    } else {
                        hardware.limiter.setPosition(Values.LIMITER_CLOSE);
                        Values.intake_Values.intakeTarget = 0;
                        Values.transfer_Values.transferTarget = 0;
                    }


                }else {
                    Values.intake_Values.intakeTarget = Values.intake_Values.intakeHold*2;
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
        Values.tx = methods.limelightCorrection(hardware.ll, dist);
        Values.turretPos = methods.AutoAim(pose,hardware.ll);
        Values.turretOverride += gamepad1.left_trigger/100;
        Values.turretOverride -= gamepad1.right_trigger/100;
        Values.turretOverride = Math.min(0.5,Math.max(-0.5,Values.turretOverride));


        intakePID.velocity_PID(hardware.intake, Values.intake_Values.intakeTarget, "intake");
        transferPID.velocity_PID(hardware.transfer, Values.transfer_Values.transferTarget, "transfer");
        if(dist>115) {
            flywheelPID.flywheelFF(hardware.flywheel1, hardware.flywheel2, Values.flywheel_Values.flywheelTarget);
        }else{
            flywheelPID.velocity_PID(hardware.flywheel1,hardware.flywheel2,Values.flywheel_Values.flywheelTarget);
        }

        hardware.turret1.setPosition(Values.turretPos);
        hardware.turret2.setPosition(Values.turretPos);
//        Values.lastTurret=Values.turretPos;

        telemetry.addData("mode",Values.mode);
        telemetry.addData("tx",Values.tx);
        telemetry.addData("team",Values.team);
        telemetry.addData("pose",pose);
        telemetry.addData("heading",Math.toDegrees(follower.getHeading())*1.0006);
        telemetry.addData("dist",dist);
        telemetry.addData("flywheel target",Values.flywheel_Values.flywheelTarget);
        telemetry.addData("flywheel rpm", String.format("1: %f,2: %f",flywheelVel1,flywheelVel2));
        telemetry.addData("flywheel power",String.format("1: %f,2: %f",hardware.flywheel1.getPower(),hardware.flywheel2.getPower()));
        telemetry.addData("Intakepower", hardware.intake.getPower());
        telemetry.addData("transferpower", hardware.transfer.getPower());
        telemetry.addData("turret position",Values.turretPos);
        telemetry.update();

    }
}