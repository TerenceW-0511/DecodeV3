package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "BreakBeam Test", group = "test")
public class test extends LinearOpMode {
    private Methods methods;

    private DigitalChannel breakBeam;
    private DigitalChannel breakBeam2;
    private DigitalChannel breakBeam3;
    private DigitalChannel breakBeam4;
    private DcMotorEx intake, transfer,flywheel1,flywheel2;


    double counter = 0;
    boolean prevBottom = false,prevTop = false;

    @Override
    public void runOpMode() {
        methods = new Methods();
        // Get the digital sensor from the hardware map
        breakBeam = hardwareMap.get(DigitalChannel.class, "breakBeam");
        breakBeam2 = hardwareMap.get(DigitalChannel.class,"breakBeam2");
        breakBeam3 = hardwareMap.get(DigitalChannel.class,"breakBeam3");
        breakBeam4 = hardwareMap.get(DigitalChannel.class,"breakBeam4");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        transfer = hardwareMap.get(DcMotorEx.class,"transfer");

        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        // Set the channel as an input
        breakBeam.setMode(DigitalChannel.Mode.INPUT);
        breakBeam2.setMode(DigitalChannel.Mode.INPUT);
        breakBeam3.setMode(DigitalChannel.Mode.INPUT);
        breakBeam4.setMode(DigitalChannel.Mode.INPUT);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setPower(0);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setPower(0);
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
        Values.counter=0;
        // Wait for the driver to press PLAY
        waitForStart();
        // Loop while the OpMode is active
        while (opModeIsActive()) {

            boolean state1 = !breakBeam.getState(); //top in
            boolean state2 = !breakBeam2.getState(); //top out
            boolean state3 = !breakBeam3.getState(); //bottom
            boolean state4 = !breakBeam4.getState(); //bottom  down

            boolean topBlocked = state1 || state2;
            boolean bottomBlocked = state3 || state4;

//            if (bottomBlocked && !prevBottom){
//                counter++;
//            }
            prevBottom = bottomBlocked;

            if (gamepad1.left_bumper) {

                intake.setPower(1);
                transfer.setPower(.6);
            } else if (gamepad1.dpad_left){
                intake.setPower(1);
                transfer.setPower(0);
            } else {
//                    hardware.limiter.setPosition(Values.LIMITER_OPEN);
                intake.setPower(0);
                transfer.setPower(0);
            }
            if(gamepad1.a){
                Values.mode = Values.Modes.SHOOTING;
            } else if (gamepad1.b) {
                Values.mode = Values.Modes.INTAKING;
            }
            methods.flywheelFFTele(flywheel1,flywheel2,1000);
            methods.countBalls(breakBeam,breakBeam2,breakBeam3,breakBeam4);
            telemetry.addData("states",String.format("1: "+state1+" 2: "+state2+" 3: "+state3+" 4: "+state4));
//            telemetry.addData("counter",counter);
            telemetry.addData("count",Values.counter);
            telemetry.addData("frames",Values.frameCount);
            telemetry.addData("mode",Values.mode);
            telemetry.update();
        }
    }
}

