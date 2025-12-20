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

    public void init(){
        hardware = new Hardware(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.update();
    }
    @Override
    public void start(){
        follower.startTeleopDrive();
    }

    @Override
    public void loop(){
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );
        if (gamepad1.aWasPressed()){
            hardware.flywheel1.setPower(1);
            hardware.flywheel2.setPower(1);
        }
        if (gamepad1.bWasPressed()){
            hardware.flywheel1.setPower(-1);
            hardware.flywheel2.setPower(-1);
        }
    }

}