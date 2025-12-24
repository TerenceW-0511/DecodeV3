package org.firstinspires.ftc.teamcode.pedroPathing.teleop;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;




@TeleOp(name = "LimelightTest",group = "Test")
public class LimelightTest extends OpMode {


    private FieldManager fieldManager;
    private Style robotStyle;
    private Limelight3A ll;

    private GoBildaPinpointDriver pinpoint;
    static final double ROBOT_SIZE = 0.45;
    static final double ROBOT_LINE = 0.35;



    @Override
    public void init() {
        fieldManager = PanelsField.INSTANCE.getField();
        fieldManager.init();
        fieldManager.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        robotStyle = new Style("Robot", "#4CAF50", 2.0);
        ll = hardwareMap.get(Limelight3A.class, "ll");
        ll.pipelineSwitch(1);
        fieldManager.update();
    }


    @Override
    public void start() {
        ll.start();
    }


    @Override
    public void loop() {
        fieldManager.getCanvas().reset();
        double pose2D = pinpoint.getHeading(AngleUnit.RADIANS);
        ll.updateRobotOrientation(pose2D);
        LLResult LLResult = ll.getLatestResult();
        if (LLResult != null && LLResult.isValid()) {
            Pose3D botpose = LLResult.getBotpose_MT2();
            Position position = botpose.getPosition();
            telemetry.addData("X", botpose.getPosition().x);
            telemetry.addData("Y", botpose.getPosition().y);
            telemetry.addData("Heading", botpose.getOrientation().getYaw(AngleUnit.DEGREES));


            double dy = botpose.getPosition().y - pinpoint.getPosY(DistanceUnit.METER);
            double dx = botpose.getPosition().x - pinpoint.getPosX(DistanceUnit.METER);

            double dist = Math.hypot(dx,dy);
            Position pos = botpose.getPosition();
            Pose botpose2D = new Pose(pos.x, pos.y, Math.toRadians(pose2D));
            drawRobot(botpose2D.getX(), botpose2D.getY(), botpose2D.getHeading());
        }
        telemetry.update();
        fieldManager.update();
        pinpoint.update();
    }
    public  void  configurePinpoint(){
        pinpoint.setOffsets(-3.07653, 2.92855, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }
    private void drawRobot(double x, double y, double heading) {
        fieldManager.setStyle(robotStyle);

        fieldManager.moveCursor(x, y);
        fieldManager.rect(ROBOT_SIZE, ROBOT_SIZE);

        fieldManager.moveCursor(x, y);
        fieldManager.line(Math.cos(heading) * ROBOT_LINE, Math.sin(heading) * ROBOT_LINE );
    }
}
