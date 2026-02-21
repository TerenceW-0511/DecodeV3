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


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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
    private boolean waitingForLimiter = false;
    private double limiterOpenTime = 0;
 //TODO: add flywheel idle, turret no auto aim

    private double lastFlywheel1Power = 999;
    private double lastFlywheel2Power = 999;
    private double lastIntakePower = 999;
    private double lastTransferPower = 999;

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
    }
    @Override
    public void init_loop(){
    }
    @Override
    public void start(){
        hardware.ll.start();
        follower.startTeleopDrive();
        timer.resetTimer();
        lastTeam = Values.team;
        hardware.ll.pipelineSwitch(Values.team == Values.Team.RED ? 1 : 2);
    }


    @Override
    public void loop() {
        follower.update();

//        Values.autonFollowerX = follower.getPose().getX();
//        Values.autonFollowerY = follower.getPose().getY();
//        Values.autonHeading = Math.toDegrees(follower.getHeading());

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x*3/4,
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
            timer.resetTimer();

        }
        if (gamepad1.rightBumperWasPressed() && Values.mode == Values.Modes. INTAKING){
            hardware.ll.reloadPipeline();
            Values.init=true;
            Values.mode = Values.Modes.SHOOTING;
            timer.resetTimer();
        }




        if (gamepad1.bWasPressed()){
            Values.tx=0;
            methods.manualRelocalize(follower);
        }
        if (gamepad1.rightStickButtonWasPressed()){
            hardware.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hardware.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (Values.team != lastTeam) {
            hardware.ll.pipelineSwitch(Values.team == Values.Team.RED ? 1 : 2);
            lastTeam = Values.team;
        }

        if (gamepad1.leftStickButtonWasPressed()){
            hardware.ll.start();
            hardware.ll.reloadPipeline();
            if (Values.team==Values.Team.RED){
                Values.team = Values.Team.BLUE;
            }else{
                Values.team=Values.Team.RED;
                }
        }


//        if (gamepad1.dpadUpWasPressed()){
//            Values.flywheel_Values.flywheelTarget+=20;
//        }else if (gamepad1.dpadDownWasPressed()){
//            Values.flywheel_Values.flywheelTarget-=20;
//        }

//        if (gamepad1.aWasPressed()){
//            Values.hoodPos += 0.02;
//        }else if (gamepad1.yWasPressed()){
//            Values.hoodPos -=0.02;
//        }
        Values.hoodPos = methods.hoodControl(follower,hardware.flywheel1,hardware.flywheel2);
        hardware.hood1.setPosition(Values.hoodPos);




        switch(Values.mode) {
            case INTAKING:
                if (Values.init && timer.getElapsedTimeSeconds()>0.4){
                    Values.init=false;
                }
                hardware.led.setPosition(0.333); //orange=
//                if (timer.getElapsedTimeSeconds()>0.1) {
//                    hardware.limiter.setPosition(Values.LIMITER_CLOSE);
//                }
                hardware.limiter.setPosition(Values.LIMITER_CLOSE);
                if (gamepad1.left_bumper && timer.getElapsedTimeSeconds()>0.5) {

                    setPowerIfChanged(hardware.intake, 1, "intake");
                    hardware.transfer.setPower(.6);
                } else {
//                    hardware.limiter.setPosition(Values.LIMITER_OPEN);
                    setPowerIfChanged(hardware.intake,  0, "intake");
                    hardware.transfer.setPower(0);

                }
                break;
            case SHOOTING:


                double rpmError = Math.abs((flywheelVel1+flywheelVel2)/2 - Values.flywheel_Values.flywheelTarget);
                hardware.limiter.setPosition(Values.LIMITER_OPEN);
                if (rpmError > 80){
                    hardware.led.setPosition(0.722); //purple
                }else{
                    hardware.led.setPosition(0.444); //green
                }

//                if (dist>120 && gamepad1.atRest()){
//                    follower.holdPoint(follower.getPose(),true);
//                }
                boolean atSpeed = rpmError < 70;
                if (atSpeed && timer.getElapsedTimeSeconds() > 0.15) {
                    setPowerIfChanged(hardware.intake, 1, "intake");
                    hardware.transfer.setPower(1);

                }else if (dist>120){
                    setPowerIfChanged(hardware.intake,0,"intake");
                    hardware.transfer.setPower(0);
                }





                break;
        }
        Values.flywheel_Values.flywheelTarget = methods.flywheelControl(follower,hardware.hood1.getPosition());

//        methods.limelightCorrection(hardware.ll, dist);
        flywheelPID.flywheelFFTele(hardware.flywheel1, hardware.flywheel2, Values.flywheel_Values.flywheelTarget);
//        if (hardware.ll.getTimeSinceLastUpdate()>500){
//            hardware.ll.reloadPipeline();
//            Values.tx=0;
//        }
        double turretEncoder = -hardware.intake.getCurrentPosition();

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

        telemetry.addData("mode",Values.mode);
        telemetry.addData("ll running",hardware.ll.isRunning());

        telemetry.addData("tx",Values.tx);

        telemetry.addData("team",Values.team);
        telemetry.addData("pose",pose);
        telemetry.addData("heading",Math.toDegrees(follower.getHeading())*1.0006);
        telemetry.addData("dist",dist);
        telemetry.addData("flywheel target",Values.flywheel_Values.flywheelTarget);
        telemetry.addData("flywheel rpm", String.format("1: %f,2: %f",flywheelVel1,flywheelVel2));
        telemetry.addData("flywheel power",String.format("1: %f,2: %f",hardware.flywheel1.getPower(),hardware.flywheel2.getPower()));
        telemetry.addData("flywheel current", String.format("1: %f, 2: %f", hardware.flywheel1.getCurrent(CurrentUnit.MILLIAMPS),hardware.flywheel2.getCurrent(CurrentUnit.MILLIAMPS)));
        telemetry.addData("hood pos",Values.hoodPos);
        telemetry.addData("turret position",turretEncoder);
        telemetry.addData("turret override",Values.turretOverride);

        telemetry.update();

    }
    private void setPowerIfChanged(DcMotorEx motor, double newPower, String motorName) {
        double lastPower;

        switch (motorName) {
            case "fly1":
                lastPower = lastFlywheel1Power;
                break;
            case "fly2":
                lastPower = lastFlywheel2Power;
                break;
            case "intake":
                lastPower = lastIntakePower;
                break;
            case "transfer":
                lastPower = lastTransferPower;
                break;
            default:
                motor.setPower(newPower);
                return;
        }

        if (Math.abs(lastPower - newPower) > 0.01) {
            motor.setPower(newPower);

            switch (motorName) {
                case "fly1": lastFlywheel1Power = newPower; break;
                case "fly2": lastFlywheel2Power = newPower; break;
                case "intake": lastIntakePower = newPower; break;
                case "transfer": lastTransferPower = newPower; break;
            }
        }
    }

}
