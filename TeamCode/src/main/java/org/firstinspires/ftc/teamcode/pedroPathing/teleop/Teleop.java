package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode {
    public Follower follower;
    public Hardware hardware;
    private Methods intakePID;
    private Methods transferPID;
    private Methods flywheelPID;


    public void init(){
        hardware = new Hardware(hardwareMap);
        intakePID = new Methods();
        transferPID = new Methods();
        flywheelPID = new Methods();
        Values.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.update();
    }
    @Override
    public void start(){
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );
//        if (gamepad1.bWasPressed()) {
//            hardware.flywheel1.setPower(-1);
//            hardware.flywheel2.setPower(-1);
//        }
//        if (gamepad1.yWasPressed()) {
//            hardware.flywheel1.setPower(0);
//            hardware.flywheel2.setPower(0);
//        }
//        if (gamepad1.left_trigger > 0.05) {
//            double intakePower = gamepad1.left_trigger;
//            hardware.intake.setPower(intakePower);
//        } else {
//            hardware.intake.setPower(0);
//        }
//        if (gamepad1.right_trigger > 0.05) {
//            double transferPower = gamepad1.right_trigger;
//            hardware.transfer.setPower(transferPower);
//        } else {
//            hardware.transfer.setPower(0);
////        }
        if (gamepad1.leftBumperWasPressed() && Values.mode== Values.Modes.SHOOTING){
            Values.mode=Values.Modes.INTAKING;

        }
        if (gamepad1.rightBumperWasPressed() && Values.mode == Values.Modes.INTAKING){
            Values.mode = Values.Modes.SHOOTING;
        }
        if (gamepad1.dpadUpWasPressed()){
            Values.flywheel_Values.flywheelVelocity+= 20;
        } else if (gamepad1.dpadDownWasPressed()) {
            Values.flywheel_Values.flywheelVelocity -=20;
        }

        if (gamepad1.yWasPressed()){
            Values.hoodPos+= 0.02;
        }else if (gamepad1.aWasPressed()){
            Values.hoodPos-=0.02;
        }
        Values.flywheel_Values.flywheelTarget = Values.flywheel_Values.flywheelVelocity;


        switch(Values.mode) {
            case INTAKING:
                hardware.limiter.setPosition(Values.LIMITER_CLOSE);
                if (gamepad1.left_bumper) {
                    Values.intake_Values.intakeTarget = Values.intake_Values.intakeIntaking;
//                    hardware.transfer.setPower(.7);
                    Values.transfer_Values.transferTarget = Values.transfer_Values.transferIntake;
                } else {
                    Values.intake_Values.intakeTarget = 0;
                    Values.transfer_Values.transferTarget = 0;
//                    hardware.transfer.setPower(0);
                }
                break;
            case SHOOTING:
                hardware.limiter.setPosition(Values.LIMITER_OPEN);
                if (gamepad1.right_bumper) {
                    Values.intake_Values.intakeTarget = Values.intake_Values.intakeIntaking;
                    if (Math.abs(hardware.flywheel2.getVelocity()-2000)<100) {
                        telemetry.addData("ready","hai");
                        hardware.transfer.setPower(1);
                        Values.transfer_Values.transferTarget = Values.transfer_Values.transferUp;
                    }else{
                        hardware.transfer.setPower(.7);
                        Values.transfer_Values.transferTarget=Values.transfer_Values.transferIntake;
                    }
                }
                break;
        }
        Values.turretPos-= gamepad1.right_trigger/30;
        Values.turretPos+= gamepad1.left_trigger/30;
        Values.turretPos=Math.min(Math.max(0,Values.turretPos),1);





        intakePID.velocity_PID(hardware.intake, Values.intake_Values.intakeTarget, "intake");
        transferPID.velocity_PID(hardware.transfer, Values.transfer_Values.transferTarget, "transfer");
        flywheelPID.velocity_PID(hardware.flywheel1,hardware.flywheel2, Values.flywheel_Values.flywheelTarget);


        hardware.turret1.setPosition(Values.turretPos);
        hardware.turret2.setPosition(Values.turretPos);
        hardware.hood1.setPosition(Values.hoodPos);

        telemetry.addData("mode",Values.mode);
        telemetry.addData("limiter",hardware.limiter.getPosition());
        telemetry.addData("flywheel 1 velocity", hardware.flywheel1.getVelocity());
        telemetry.addData("flywheel 2 velocity", hardware.flywheel2.getVelocity());
        telemetry.addData("flywheel 1 power",hardware.flywheel1.getPower());
        telemetry.addData("flywheel 2 power",hardware.flywheel2.getPower());
        telemetry.addData("flywheel target", Values.flywheel_Values.flywheelTarget);
        telemetry.addData("Intakepower", hardware.intake.getPower());
        telemetry.addData("transferpower", hardware.transfer.getPower());
        telemetry.addData("turret pos",Values.turretPos);
        telemetry.update();

    }
}