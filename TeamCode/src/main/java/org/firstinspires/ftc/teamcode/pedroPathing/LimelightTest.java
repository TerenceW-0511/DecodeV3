package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Disabled
@TeleOp(name = "LimelightTest",group = "Test")
public class LimelightTest extends OpMode {


    private FieldManager fieldManager;
    private Style robotStyle;
    private Limelight3A ll;
    private Follower follower;

    static final double ROBOT_SIZE = 0.45;
    static final double ROBOT_LINE = 0.35;



    @Override
    public void init() {
        fieldManager = PanelsField.INSTANCE.getField();
        fieldManager.init();
        fieldManager.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(129,113.221415,Math.toRadians(180)));


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
        follower.update();
        fieldManager.getCanvas().reset();
        ll.updateRobotOrientation(follower.getHeading());
        LLResult LLResult = ll.getLatestResult();
        if (LLResult != null && LLResult.isValid()) {
            Pose3D botpose = LLResult.getBotpose_MT2();
            Pose convertedPose = fromPose3d(botpose);

            double x = toInches(convertedPose.getX());
            double y = toInches(convertedPose.getY());

            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Heading", botpose.getOrientation().getYaw(AngleUnit.DEGREES));


            drawRobot(x, y, follower.getHeading());
            drawRobot(follower.getPose().getX(),follower.getPose().getY(),follower.getHeading());
        }
        telemetry.update();
        fieldManager.update();
    }

    private void drawRobot(double x, double y, double heading) {
        fieldManager.setStyle(robotStyle);

        fieldManager.moveCursor(x, y);
        fieldManager.rect(ROBOT_SIZE, ROBOT_SIZE);

        fieldManager.moveCursor(x, y);
        fieldManager.line(Math.cos(heading) * ROBOT_LINE, Math.sin(heading) * ROBOT_LINE );
    }
    public static Pose fromPose3d(Pose3D original) {
        Pose pose = new Pose(
                toInches(original.getPosition().y) + 72,
                -toInches(original.getPosition().x) + 72,
                original.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI / 2
        );
        return pose.setHeading((pose.getHeading() + Math.PI) % (2 * Math.PI));
    }
    public static double toInches(double meters){
        return meters*39.3701;
    }
}