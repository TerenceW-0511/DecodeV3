package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "FlywheelVelocityTuner", group = "Tuning")
public class VelocityTuner extends OpMode {

    public static double targetVelocity = 1500;
    public static String motorType = "intake";
    public static double kp=0,ki=0,kd=0,kf=0;
    PIDFController pidf = new PIDFController(0,0,0,0);

    private DcMotorEx flywheel1,flywheel2;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Methods methods = new Methods();

    @Override
    public void init(){

        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel1.setPower(0);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double target = targetVelocity;

        double currentVel = (flywheel1.getVelocity() + flywheel2.getVelocity()) / 2.0;
        double error = target - currentVel;
        pidf.setPIDF(kp,ki,kd,kf);


        double power = pidf.calculate(currentVel,target);

        flywheel1.setPower(power);
        flywheel2.setPower(power);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Vel", target);
        packet.put("Current Vel", currentVel);
        packet.put("Error", error);
        packet.put("Power", power);
        dashboard.sendTelemetryPacket(packet);
    }
}