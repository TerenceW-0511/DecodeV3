package org.firstinspires.ftc.teamcode.pedroPathing;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.math.RoundingMode;

@Config
public class Methods {
    private boolean firstLoop = true;
    private double lastPos;
    private double lastTime;
    private double lastTarget=0;
    public static final double rpmAdj = 20;
    public static double k=-0.00015;
    private double lastLLError = 0;
    private long lastLLTime = System.nanoTime();
    public static double yawScalar = 1.0006;


    public static double offsetAmt = 0;
    private double integral = 0.0;
    private double prevError = 0.0;
    private long prevTime = System.nanoTime();
    public static double test = -15;
    public static double hoodBase = 0.5;
    private double a= 2558.3597233948644,b=-16.387017585894974,c=-2356.4076710655913,d=0.08890184449457195,e=1117.6724685437139,f=16.196925315213786;
    public double filteredX=0,aprilx = 0;
    private double lastFly1Power = 999;
    private double lastFly2Power = 999;
    private double lastFlywheelTarget = 0;
    private double lastFlywheelError = 0;
    private double lastFlywheelTime = System.nanoTime() / 1e9;


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


    public double flywheelFFTele(DcMotorEx m1, DcMotorEx m2, double target){
//
//        pidf.setPIDF(fP,fI,fD,fF);
        double curr = (m1.getVelocity()+m2.getVelocity())/2;
        double power = Values.flywheel_Values.fV*target + Values.flywheel_Values.fS +Values.flywheel_Values.flywheelPIDController.calculate(curr,target);
        m1.setPower(power);
        m2.setPower(power);
        return curr;
    }
//    public double flywheelFF(
//            DcMotorEx m1,
//            DcMotorEx m2,
//            double targetVel // ticks/sec
//    ) {
//        double kS = Values.flywheel_Values.kS;
//        double kV = Values.flywheel_Values.kV;
//        double kP = Values.flywheel_Values.kP;
//
//        // Average measured velocity
//        double currentVel = (m1.getVelocity() + m2.getVelocity()) / 2.0;
//
//        if (targetVel == 0) {
//            m1.setPower(0);
//            m2.setPower(0);
//            return 0;
//        }
//
//        double error = targetVel - currentVel;
//
//        double ff = kV * targetVel + kS * Math.signum(targetVel);
//        double p  = kP * error;
//
//        double power = ff + p;
//        power = Range.clip(power, -1.0, 1.0);
//
//        m1.setPower(power);
//        m2.setPower(power);
//
//        return currentVel;
//    }



    public double turretPID(double curr, double target) {

        if (Math.abs(curr-target)<50){
            return 0.5;
        }
        Values.turret_Values.turretPIDController.setPIDF(Values.turret_Values.kP,Values.turret_Values.kI,Values.turret_Values.kD,0);
        double power = Values.turret_Values.turretPIDController.calculate(curr,target+Values.llOverride);
        power = Range.clip(power, -1.0, 1.0);

        return (power + 1) / 2;
    }





    public void resetPID() {
        firstLoop = true;
    }

    public double getDist(Pose pose){
        if (Values.team==Values.Team.BLUE){
            return Math.hypot((pose.getX()-12.5),(pose.getY()-137.3));
        }else{
            return Math.hypot((pose.getX()-131.5),(pose.getY()-137.5));
        }
    }


    public void manualRelocalize(Follower follower){

        if (Values.team==Values.Team.BLUE) {
            follower.setPose(new Pose(135-23.5, 6.5, Math.toRadians(0)));
        }else{
            follower.setPose(new Pose(9+23.5,6.5,Math.toRadians(180)));
        }
        filteredX=0; 
        Values.turretOverride=0;
        Values.tx=0;
        Values.llOverride=0;
    }
    public String limelightRelocalize(Limelight3A ll, Follower follower,Servo turret){
        LLResult result = ll.getLatestResult();
        if (!ll.isConnected()) return "not connected";
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
    public static Pose sotm(Follower follower){
        Vector velocity = follower.getVelocity();
        Pose curr = follower.getPose();
        Pose predicted = new Pose(
                curr.getX() + velocity.getXComponent(),
                curr.getY()+ velocity.getYComponent()
        );
        return predicted;
    }
    public double AutoAim(Pose botPose, Limelight3A ll) {
        double dx, dy, alpha;

        if (Values.team == Values.Team.BLUE) {
            dx = botPose.getX() - Values.blueGoal.getX();
            dy = Values.blueGoal.getY() - botPose.getY();
            alpha = 180
                    - Math.toDegrees(botPose.getHeading())
                    - Math.toDegrees(Math.atan2(dy, dx));
        } else {
            dx = botPose.getX() - Values.redGoal.getX();
            dy = Values.redGoal.getY() - botPose.getY();
            alpha = 180
                    - Math.toDegrees(botPose.getHeading())
                    - Math.toDegrees(Math.atan2(dy, dx));
        }
        LLResult result= ll.getLatestResult();
        double dist = getDist(botPose);
        if (dist > 120) {
            offsetAmt = (Values.team == Values.Team.RED) ? -1 : 3;
        }else{
            offsetAmt = 0;
        }
        if (!result.isValid()){
            Values.tx=0;
        }else {
            Values.tx = result.getTx() - offsetAmt;
        }
        filteredX+=Values.tx*test;
        double target =filteredX+alpha * 1725/18;


        double wrapped = (target+17000)%34000;
        if (wrapped<0) wrapped+=34000;
        wrapped-=17000;
        return wrapped;
    }

    public double limelightOffset(Pose botPose){
        double x = botPose.getX();
        double y = botPose.getY();
        if (Values.team == Values.Team.BLUE) {
            double alpha = Math.toDegrees(Math.atan2(x - Values.blueTag.getX(), Values.blueTag.getY() - y));
            double lambda = Math.toDegrees(Math.atan2(Values.blueGoal.getY() - y, x - Values.blueGoal.getX()));
            return 90-alpha-lambda;
        }else{
            double alpha = Math.toDegrees(Math.atan2(Values.redTag.getX()-x,Values.redTag.getY()-y));
            double lambda = Math.toDegrees(Math.atan2(Values.redGoal.getY()-y,Values.redGoal.getX()-x));
            return 90-alpha-lambda;
        }

    }


    public double limelightCorrection(Limelight3A ll, double dist) {
        LLResult result = ll.getLatestResult();

        if (!ll.isConnected() || !result.isValid()) {
            return Values.tx; // or set to 0 if you want
        }

        Values.tx = result.getTx();

        double offsetAmt = 0;

        if (dist > 120) {
            offsetAmt = (Values.team == Values.Team.RED) ? 0.5 : -0.5;
        }

        double tx = result.getTx() - offsetAmt;

        Values.tx = tx;

        if (Math.abs(tx) < 2) return tx;

        return tx;




//        double targetOffset = 0.0;
//        if (dist > 120) {
//            targetOffset = (Values.team == Values.Team.RED)
//                    ? offsetAmt
//                    : -offsetAmt;
//        }
//
//        double error = tx - targetOffset;
//
//        long now = System.nanoTime();
//        double dt = (now - prevTime) / 1e9;
//        prevTime = now;
//        if (dt <= 0) dt = 1e-3;
//
//        integral += error * dt;
//        double derivative = (error - prevError) / dt;
//        prevError = error;
//
//        double output = kp * error
//                + ki * integral
//                + kd * derivative;
//
//        if (Math.abs(error) > 0.5) {
//            Values.Override = output;
//            output = Math.max(-maxSpeed, Math.min(maxSpeed, output));
//        } else {
//            integral = 0;
//            output = 0;
//        }
//
//        return new double[]{tx, targetOffset, output};
    }
    public static double normalizeDeg(double angle) {
        angle %= 360;
        if (angle >= 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
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


    public double hoodControl(Follower follower,DcMotorEx flywheel1,DcMotorEx flywheel2){
//        double targetVel = flywheelControl(follower,1);

        double rpmError = Math.abs(Values.flywheel_Values.flywheelTarget - (flywheel1.getVelocity()+flywheel2.getVelocity())/2);
        double dist = getDist(follower.getPose());
        if (dist>120) {
            return hoodNominal(dist) + rpmError * k;
        }else{
            return hoodNominal(dist);
        }
    }

    //x = distance, y = hood
    //flywheel = 2024.460854150586-5.521875372212847x-2088.3394044791553y+0.04960697997872554x^2+1095.182079053327y^2+14.997680959492307xy
    public double flywheelControl(Follower follower, double hoodPos){
        double x = getDist(follower.getPose());
        double y = hoodPos;
        return a
                + b * x
                + c * y
                + d * x * x
                + e * y * y
                + f * x * y
                ;
    }

}
/// turret servo angle LUT
/// ll: relocalization