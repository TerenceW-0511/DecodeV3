package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
@TeleOp(name = "BreakBeam Test", group = "test")
public class test extends LinearOpMode {

    private DigitalChannel breakBeam;
    private DigitalChannel breakBeam2;

    @Override
    public void runOpMode() {
        // Get the digital sensor from the hardware map
        breakBeam = hardwareMap.get(DigitalChannel.class, "breakBeam");
        breakBeam2 = hardwareMap.get(DigitalChannel.class,"breakBeam2");

        // Set the channel as an input
        breakBeam.setMode(DigitalChannel.Mode.INPUT);
        breakBeam.setMode(DigitalChannel.Mode.OUTPUT);

        // Wait for the driver to press PLAY
        waitForStart();

        // Loop while the OpMode is active
        while (opModeIsActive()) {
            boolean stateHigh = breakBeam.getState();
            boolean detected = stateHigh;

            if (detected) {
                telemetry.addLine("Object detected!");
            } else {
                telemetry.addLine("No object detected");
            }

            telemetry.addData("Raw (HIGH/LOW)", stateHigh);
            boolean stateHigh2 = breakBeam2.getState();
            boolean detected2 = stateHigh2;

            if (detected) {
                telemetry.addLine("Object detected!");
            } else {
                telemetry.addLine("No object detected");
            }

            telemetry.addData("Raw (HIGH/LOW)", stateHigh2);
            telemetry.update();
        }
    }
}
