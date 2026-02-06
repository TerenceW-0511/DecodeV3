package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "ServoTuner", group = "Tuning")
public class ServoTuner extends OpMode {

    public static double targetposition = 0;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    private Servo turret1, turret2;
    private DcMotorEx intake;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Methods methods = new Methods();
    double integral = 0;
    double lastError = 0;
    @Override
    public void init(){
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        intake = hardwareMap.get(DcMotorEx.class,"intake");

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        double target = targetposition;

        double currentPos = intake.getCurrentPosition();
        double error = target - currentPos;

        double power;
//        double ff = kv * target
//                + ks * Math.signum(target);
        double derivative = error - lastError;
        lastError = error;
        integral += error;
        double p = kp * error;
        double i = ki * integral;
        double d = kd * derivative;
        power = p + i + d;
        double newPower = Range.clip(power, -1.0, 1.0);
        double convertedPower = (newPower+1)/2;
        turret1.setPosition(convertedPower);
        turret2.setPosition(convertedPower);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Pos", target);
        packet.put("Current Pos", currentPos);
        packet.put("Error", error);
        packet.put("Power", convertedPower);
        dashboard.sendTelemetryPacket(packet);
    }
}