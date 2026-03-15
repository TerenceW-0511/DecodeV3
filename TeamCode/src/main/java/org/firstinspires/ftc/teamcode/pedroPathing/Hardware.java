package org.firstinspires.ftc.teamcode.pedroPathing;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    public Limelight3A ll;
    public DcMotorEx intake, transfer, flywheel1, flywheel2;
    public Servo led;
    public DigitalChannel breakBeamIntake;
    public DigitalChannel breakBeamIntake2;

    public DigitalChannel breakBeamOutake;

    public DigitalChannel breakBeamOutake2;

    public Servo hood1, limiter, turret1, turret2, kicker;

    public Hardware(HardwareMap hardwareMap) {
        ll = hardwareMap.get(Limelight3A.class, "ll");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        hood1 = hardwareMap.get(Servo.class, "hood");
        limiter = hardwareMap.get(Servo.class,"limiter");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        breakBeamIntake = hardwareMap.get(DigitalChannel.class,"breakBeam1");
        breakBeamIntake2 = hardwareMap.get(DigitalChannel.class,"breakBeam2");
        breakBeamOutake = hardwareMap.get(DigitalChannel.class, "breakBeam3");
        breakBeamOutake2 = hardwareMap.get(DigitalChannel.class, "breakBeam4");

        breakBeamIntake.setMode(DigitalChannel.Mode.INPUT);
        breakBeamIntake.setState(false);
        breakBeamIntake2.setMode(DigitalChannel.Mode.INPUT);
        breakBeamIntake2.setState(false);
        breakBeamOutake.setMode(DigitalChannel.Mode.INPUT);
        breakBeamOutake.setState(false);
        breakBeamOutake2.setMode(DigitalChannel.Mode.INPUT);
        breakBeamOutake2.setState(false);
        led = hardwareMap.get(Servo.class, "led");
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel1.setPower(0);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setPower(0);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setPower(0);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setPower(0);

    }
}