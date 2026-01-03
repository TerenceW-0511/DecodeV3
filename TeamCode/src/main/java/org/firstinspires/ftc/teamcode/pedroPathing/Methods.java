package org.firstinspires.ftc.teamcode.pedroPathing;
import com.arcrobotics.ftclib.controller.PIDFController;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Map;

public class Methods {
    private boolean firstLoop = true;
    private double lastPos;
    private double lastTime;
    private double lastTarget=0;
    public static final double k=0.0002;
    private double a= 1810.2766439035408,b=-3.449090076831704,c=-724.2250766458692,d=0.04813077393542908,e=224.62956712218775,f=4.943595431803776;

    public double velocity_PID(DcMotorEx motor, double targetVelocity, String mode) {
        PIDFController controller;
        double kF, kD, kP,kI;

        switch (mode) {
            case "intake":
                kI = Values.intake_Values.iI;
                kD = Values.intake_Values.iD;
                kP = Values.intake_Values.iP;
                kF = Values.intake_Values.iK;
                controller = Values.intake_Values.intakePIDController;

                break;
            case "transfer":
                kI = Values.transfer_Values.trI;
                kD = Values.transfer_Values.trD;
                kP = Values.transfer_Values.trP;
                kF = Values.transfer_Values.trF;
                controller = Values.transfer_Values.transferPIDController;
                break;
            case "flywheel1":
                kI = Values.flywheel_Values.fI;
                kD = Values.flywheel_Values.fD;
                kP = Values.flywheel_Values.fP;
                kF = Values.flywheel_Values.fF;
                controller = Values.flywheel_Values.flywheelPIDController;
                break;

            default:
                throw new IllegalArgumentException("Error: " + mode);
        }

        if (firstLoop) {
            lastPos = motor.getCurrentPosition();
            lastTime = System.nanoTime() / 1e9;
            firstLoop = false;
            return 0;
        }

        double currentPos = motor.getCurrentPosition();
        double currentTime = System.nanoTime() / 1e9;

        double dt = currentTime - lastTime;
        double dp = currentPos - lastPos;

        double measuredVelocity = dp / dt;



        lastPos = currentPos;
        lastTime = currentTime;

        if (targetVelocity == 0) {
            controller.reset();
            motor.setPower(0);
            lastTarget = 0;
            return 0;
        }

        if (targetVelocity != lastTarget) {
            controller.reset();
            lastTarget = targetVelocity;
        }

        controller.setPIDF(kP, kI, kD, kF);
        double power = controller.calculate(motor.getVelocity(), targetVelocity);
        motor.setPower(power);
        return measuredVelocity;

    }
    public double velocity_PID(DcMotorEx motor,DcMotorEx motor2, double targetVelocity) {
        PIDFController controller;
        double kF, kD, kP,kI;

        kI = Values.flywheel_Values.fI;
        kD = Values.flywheel_Values.fD;
        kP = Values.flywheel_Values.fP;
        kF = Values.flywheel_Values.fF;
        controller = Values.flywheel_Values.flywheelPIDController;

        if (firstLoop) {
            lastPos = motor.getCurrentPosition();
            lastTime = System.nanoTime() / 1e9;
            firstLoop = false;
            return 0;
        }
        double currentPos = (double) (motor.getCurrentPosition() + motor2.getCurrentPosition()) /2;
        double currentTime = System.nanoTime() / 1e9;

        double dt = currentTime - lastTime;
        double dp = currentPos - lastPos;

        double measuredVelocity = dp / dt;



        lastPos = currentPos;
        lastTime = currentTime;

        if (targetVelocity == 0) {
            motor.setPower(0);
            return 0;
        }

        controller.setPIDF(kP, kI, kD, kF);
        double power = controller.calculate( (motor.getVelocity() + motor2.getVelocity()) /2, targetVelocity);
        motor.setPower(power);
        motor2.setPower(power);
        return measuredVelocity;

    }


    public void resetPID() {
        firstLoop = true;
    }

    public double getDist(Follower follower){
        if (Values.team==Values.Team.BLUE){
            return Math.hypot((follower.getPose().getX()-12.5),(follower.getPose().getY()-137.3));
        }else{
            return Math.hypot((follower.getPose().getX()-131.5),(follower.getPose().getY()-137.5));
        }
    }


    public void manualRelocalize(Follower follower){
        follower.setPose(new Pose(135,6.5,Math.toRadians(0)));
    }
    public String limelightRelocalize(Limelight3A ll, Follower follower,Servo turret){
        LLResult result = ll.getLatestResult();
        if (!result.isValid()) return "invalid result";
        ll.updateRobotOrientation(Math.toDegrees(result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)));
        Pose3D turretCenterPose = result.getBotpose_MT2();
        Pose pedroPose = fromPose3d(turretCenterPose);
        double turretAngle = 3600/10.128*(turret.getPosition()-0.5);
        double x = turretCenterPose.getPosition().x + 9.3393*Math.sin(Math.toRadians(-turretAngle));
        double y = turretCenterPose.getPosition().y + 9.3393*Math.cos(Math.toRadians(-turretAngle));
        Pose botPose = new Pose(x,y, pedroPose.getHeading());
        return botPose.toString();
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

    public double AutoAim(Pose botPose){
        double dy,dx,alpha;
        if (Values.team==Values.Team.BLUE) {
            dx = botPose.getX() - 12.5;
            dy = 137.3 - botPose.getY();
            alpha = 180 - Math.toDegrees(botPose.getHeading())
                    - Math.toDegrees(Math.atan2(dy, dx));
        }else{
            dx = botPose.getX() - 131.5;
            dy = 137.3 - botPose.getY();
            alpha = 180-Math.toDegrees(botPose.getHeading())
                    - Math.toDegrees(Math.atan2(dy, dx));
        }
        double servoAngle = 10.128*alpha/3600 + 0.5;
        if (servoAngle>1 || servoAngle<0){
            Values.turretDeadSpot=true;
        }else{
            Values.turretDeadSpot=false;
        }
        servoAngle = Math.min(1,Math.max(0,servoAngle));
        return 1-servoAngle;

    }

    public static double hoodNominal(double dist) {

        if (dist <= Values.hoodLUT.firstKey()) return Values.hoodLUT.firstEntry().getValue();
        if (dist >= Values.hoodLUT.lastKey())  return Values.hoodLUT.lastEntry().getValue();

        Map.Entry<Double, Double> lower = Values.hoodLUT.floorEntry(dist);
        Map.Entry<Double, Double> upper = Values.hoodLUT.ceilingEntry(dist);

        if (lower == null || upper == null) {
            return Values.hoodLUT.lastEntry().getValue();
        }

        if (lower.getKey().equals(upper.getKey())) {
            return lower.getValue();
        }

        double x0 = lower.getKey();
        double y0 = lower.getValue();
        double x1 = upper.getKey();
        double y1 = upper.getValue();

        double t = (dist - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    public double hoodControl(double dist,DcMotorEx flywheel1,DcMotorEx flywheel2){
        double rpmError = Values.flywheel_Values.flywheelTarget - (flywheel1.getVelocity()+flywheel2.getVelocity())/2.;

        double hoodComp = k * rpmError;
        return hoodNominal(dist) + hoodComp;
    }

    //x = distance, y = hood
    //flywheel = 2024.460854150586-5.521875372212847x-2088.3394044791553y+0.04960697997872554x^2+1095.182079053327y^2+14.997680959492307xy
    public double flywheelControl(Follower follower, Servo hood){
        double x = getDist(follower);
        double y = hood.getPosition();
        return a
                +b*x
                +c*y
                +d*x*x
                +e*y*y
                +f*x*y;
    }

}
/// intake: PID,velocity
/// Hood: PID,position
/// flywheel: PID velocity
/// transfer: PID?, autoaim turret
/// ll: relocalization