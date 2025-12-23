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
////        }
        if (gamepad1.leftBumperWasPressed() && Values.mode== Values.Modes.SHOOTING){
            Values.mode=Values.Modes.INTAKING;

        }
        if (gamepad1.rightBumperWasPressed() && Values.mode == Values.Modes.INTAKING){
            Values.mode = Values.Modes.SHOOTING;
        }


        switch(Values.mode) {
            case INTAKING:
                hardware.limiter.setPosition(Values.LIMITER_CLOSE);
                Values.flywheel_Values.flywheelTarget = 0;
                if (gamepad1.left_bumper) {
                    Values.intake_Values.intakeTarget = Values.intake_Values.intakeIntaking;
                    Values.transfer_Values.transferTarget = Values.transfer_Values.transferIntake;
                } else {
                    Values.intake_Values.intakeTarget = 0;
                    Values.transfer_Values.transferTarget = 0;
                }
                break;
            case SHOOTING:
                hardware.limiter.setPosition(Values.LIMITER_OPEN);
                Values.flywheel_Values.flywheelTarget = Values.flywheel_Values.flywheelVelocity;
                if (gamepad1.right_bumper) {
                    Values.intake_Values.intakeTarget = Values.intake_Values.intakeIntaking;
                    if (Math.abs(hardware.flywheel2.getVelocity()-Values.flywheel_Values.flywheelTarget)<100) {
                        telemetry.addData("ready","hai");
                        Values.transfer_Values.transferTarget = Values.transfer_Values.transferUp;
                    }else{
                        Values.transfer_Values.transferTarget=Values.transfer_Values.transferIntake;
                    }
                }
                break;
        }
        Values.turretPos+= gamepad1.right_trigger/10;
        Values.turretPos-= gamepad1.left_trigger/10;
        Values.turretPos=Math.min(Math.max(0,Values.turretPos),1);





        methods.velocity_PID(hardware.intake,Values.intake_Values.intakeTarget,"intake");
        methods.velocity_PID(hardware.transfer,Values.transfer_Values.transferTarget,"transfer");
        methods.velocity_PID(hardware.flywheel2,Values.flywheel_Values.flywheelTarget,"flywheel2");

        hardware.turret1.setPosition(Values.turretPos);
        hardware.turret2.setPosition(Values.turretPos);

        telemetry.addData("mode",Values.mode);
        telemetry.addData("limiter",hardware.limiter.getPosition());
        telemetry.addData("flywheel 1 velocity", hardware.flywheel1.getVelocity());
        telemetry.addData("flywheel 2 velocity", hardware.flywheel2.getVelocity());

        telemetry.addData("flywheel target", Values.flywheel_Values.flywheelTarget);
        telemetry.addData("Intakepower", hardware.intake.getPower());
        telemetry.addData("transferpower", Values.transfer_Values.transferTarget);
        telemetry.update();

    }
}