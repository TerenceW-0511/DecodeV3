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

    // --- Stability tuning ---
    public static double tolerance = 20;      // encoder ticks
    public static double stableTime = 0.25;   // seconds required stable

    private Servo turret1, turret2;
    private DcMotorEx intake;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private double integral = 0;
    private double lastError = 0;

    private double stableTimer = 0;

    // Loop time (you said ~0.07s)
    private final double dt = 0.07;

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

        // --- PID with real time ---
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = kp * error + ki * integral + kd * derivative;

        double clipped = Range.clip(power, -1.0, 1.0);
        double servoPos = (clipped + 1) / 2.0;

        turret1.setPosition(servoPos);
        turret2.setPosition(servoPos);

        // --- Stability Detection ---
        boolean inTolerance = Math.abs(error) < tolerance;

        if (inTolerance) {
            stableTimer += dt;
        } else {
            stableTimer = 0;
        }

        boolean stable = stableTimer >= stableTime;

        // --- Dashboard ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Pos", target);
        packet.put("Current Pos", currentPos);
        packet.put("Error", error);
        packet.put("Integral", integral);
        packet.put("Derivative", derivative);
        packet.put("Servo Pos", servoPos);
        packet.put("Stable Time", stableTimer);
        packet.put("Stable?", stable);

        dashboard.sendTelemetryPacket(packet);
    }
}
