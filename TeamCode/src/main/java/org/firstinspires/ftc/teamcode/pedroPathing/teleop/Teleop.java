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
    private Methods methods;

    public void init(){
        hardware = new Hardware(hardwareMap);
        methods = new Methods();
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
//        }
        if (gamepad1.leftBumperWasPressed() && Values.mode== Values.Modes.SHOOTING){
            Values.mode=Values.Modes.INTAKING;
            hardware.limiter.setPosition(Values.LIMITER_CLOSE);
        }
        if (gamepad1.rightBumperWasPressed() && Values.mode == Values.Modes.INTAKING){
            Values.mode = Values.Modes.SHOOTING;
        }
        switch(Values.mode){
            case INTAKING:
                if (gamepad1.left_bumper){
                    methods.velocity_PID(hardware.intake,Values.intake_Values.intakeTarget,"intake");
                    methods.velocity_PID(hardware.transfer,Values.transfer_Values.transferIntake,"transfer");
                }else{
                    hardware.intake.setPower(0);
                    hardware.transfer.setPower(0);
                }
                break;
            case SHOOTING:
                if (gamepad1.right_bumper){
                    methods.velocity_PID(hardware.flywheel1,hardware.flywheel2,Values.flywheel_Values.flywheelVelocity);
                }else {
                    hardware.flywheel1.setPower(0);
                    hardware.flywheel2.setPower(0);
                }
                if (Math.abs(hardware.flywheel1.getVelocity()-Values.flywheel_Values.flywheelVelocity)<30){
                    hardware.limiter.setPosition(Values.LIMITER_OPEN);
                }

        }

        telemetry.addData("Flywhelvelocity2", hardware.flywheel2.getVelocity());
        telemetry.addData("Flywheelvelocity1", hardware.flywheel1.getVelocity());
        telemetry.addData("Intakepower", hardware.intake.getPower());
        telemetry.addData("transferpower", hardware.transfer.getPower());
        telemetry.update();

    }
}