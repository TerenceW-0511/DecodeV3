package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

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
    private boolean speedFirstLoop = true;



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
        hardware.turret1.setPosition(Values.turretPos);
        hardware.turret2.setPosition(Values.turretPos);
        hardware.kicker.setPosition(Values.KICKER_DOWN);
    }
    @Override
    public void start(){

        follower.startTeleopDrive();
        timer.resetTimer();
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
        double dist = methods.getDist(follower);

        //MODE SWITCHING
        if (gamepad1.leftBumperWasPressed() && Values.mode== Values.Modes.SHOOTING){
            Values.mode=Values.Modes.INTAKING;

        }
        if (gamepad1.rightBumperWasPressed() && Values.mode == Values.Modes.INTAKING){
            Values.mode = Values.Modes.SHOOTING;
            timer.resetTimer();
        }


//        if (gamepad1.yWasPressed()){
//            Values.hoodPos+= 0.03;
//        }else if (gamepad1.aWasPressed()){
//            Values.hoodPos-=0.03;
//        }

        if (gamepad1.leftStickButtonWasPressed()){
            methods.manualRelocalize(follower);
        }




        Values.flywheel_Values.flywheelTarget=methods.flywheelControl(follower,hardware.hood1);
        hardware.hood1.setPosition(methods.hoodControl(dist, hardware.flywheel1, hardware.flywheel2));


        switch(Values.mode) {
            case INTAKING:
                hardware.limiter.setPosition(Values.LIMITER_CLOSE);
                hardware.kicker.setPosition(Values.KICKER_DOWN);
                if (gamepad1.left_bumper) {
                    Values.intake_Values.intakeTarget = Values.intake_Values.intakeIntaking;
//                    hardware.transfer.setPower(.7);
                    Values.transfer_Values.transferTarget = Values.transfer_Values.transferIntake;
                } else {
                    Values.intake_Values.intakeTarget = Values.intake_Values.intakeHold;
                    Values.transfer_Values.transferTarget = 0;
//                    hardware.transfer.setPower(0);
                }
                break;
            case SHOOTING:

                if (gamepad1.right_bumper && !Values.turretDeadSpot) {
                    Values.intake_Values.intakeTarget = Values.intake_Values.intakeIntaking;
                    if (dist>120) {
                        if (Math.abs(hardware.flywheel2.getVelocity() - Values.flywheel_Values.flywheelTarget) < 50) {
                            speedFirstLoop = false;
                            hardware.limiter.setPosition(Values.LIMITER_OPEN);
                            Values.transfer_Values.transferTarget = Values.transfer_Values.transferUp;
                        } else {
                            hardware.limiter.setPosition(Values.LIMITER_CLOSE);
                            Values.transfer_Values.transferTarget = 0;
                        }
                    }
                    else{
                        hardware.limiter.setPosition(Values.LIMITER_OPEN);
                        if (Math.abs(hardware.flywheel2.getVelocity() - Values.flywheel_Values.flywheelTarget) < 100) {
                            speedFirstLoop = false;
                            Values.transfer_Values.transferTarget = Values.transfer_Values.transferUp;
                        } else {
                            Values.transfer_Values.transferTarget = Values.transfer_Values.transferIntake;
                        }
                    }

                }else {
                    Values.intake_Values.intakeTarget = Values.intake_Values.intakeHold;
                }

                if (speedFirstLoop){
                    if (timer.getElapsedTimeSeconds()>2){
                        hardware.kicker.setPosition(Values.KICKER_UP);
                    }else{
                        hardware.kicker.setPosition(Values.KICKER_DOWN);
                    }
                }else if(dist>120){
                    if (timer.getElapsedTimeSeconds()>1.6){
                        hardware.kicker.setPosition(Values.KICKER_UP);
                    }else{
                        hardware.kicker.setPosition(Values.KICKER_DOWN);
                    }
                }else{
                    if (timer.getElapsedTimeSeconds()>1.2){
                        hardware.kicker.setPosition(Values.KICKER_UP);
                    }else{
                        hardware.kicker.setPosition(Values.KICKER_DOWN);
                    }
                }


                break;
        }
//        Values.turretPos-= gamepad1.right_trigger/30;
//        Values.turretPos+= gamepad1.left_trigger/30;
//        Values.turretPos=Math.min(Math.max(0,Values.turretPos),1);
        if (timer.getElapsedTimeSeconds()>0.5) {
            Values.turretPos = methods.AutoAim(follower.getPose());
        }
        Values.turretOverride += gamepad1.left_trigger/1000;
        Values.turretOverride -= gamepad1.right_trigger/1000;
        Values.turretOverride = Math.min(0.5,Math.max(-0.5,Values.turretOverride));


        intakePID.velocity_PID(hardware.intake, Values.intake_Values.intakeTarget, "intake");
        transferPID.velocity_PID(hardware.transfer, Values.transfer_Values.transferTarget, "transfer");
        flywheelPID.velocity_PID(hardware.flywheel1,hardware.flywheel2, Values.flywheel_Values.flywheelTarget);


        hardware.turret1.setPosition(Values.turretPos);
        hardware.turret2.setPosition(Values.turretPos);

        telemetry.addData("mode",Values.mode);
        telemetry.addData("limelight",methods.limelightRelocalize(hardware.ll,follower,hardware.turret1));
        telemetry.addData("timer",timer.getElapsedTimeSeconds());
        telemetry.addData("pose",follower.getPose());
        telemetry.addData("dist",dist);
        telemetry.addData("flywheel target",Values.flywheel_Values.flywheelTarget);
        telemetry.addData("flywheel rpm", String.format("1: %f,2: %f",hardware.flywheel1.getVelocity(),hardware.flywheel2.getVelocity()));
        telemetry.addData("flywheel power",String.format("1: %f,2: %f",hardware.flywheel1.getPower(),hardware.flywheel2.getPower()));
        telemetry.addData("Intakepower", hardware.intake.getPower());
        telemetry.addData("transferpower", hardware.transfer.getPower());
        telemetry.addData("dead zone",Values.turretDeadSpot);
        telemetry.update();

    }
}