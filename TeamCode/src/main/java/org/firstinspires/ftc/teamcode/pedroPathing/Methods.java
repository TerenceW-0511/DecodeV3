package org.firstinspires.ftc.teamcode.pedroPathing;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Map;

@Config
public class Methods {
    private Hardware hardware;

    private boolean firstLoop = true;
    private double lastPos;
    private double lastTime;
    private double lastTarget=0;
    public static final double rpmAdj = 20;
    public static double k=-0;


    boolean last1 = false;

    boolean last2 = false;
    private double lastLLError = 0;
    private long lastLLTime = System.nanoTime();
    public static double yawScalar = 1.0006;

    private double integral = 0.0;
    private double prevError = 0.0;
    private long prevTime = System.nanoTime();
    public static double test = -15;
    public static double weight = 1;
    public static double hoodBasex = 0.5;
    private double a = -761.0731094855594 ,b= 46.74302160590927 ,c= 3492.9369208740777 ,d= -0.1597352593405006 ,e= -1281.088539198119 ,f= -45.054672346900006;
    public double filteredX=0,aprilx = 0;
    private double lastFly1Power = 999;
    private double lastFly2Power = 999;
    private double lastFlywheelTarget = 0;
    private double lastFlywheelError = 0;
    private double lastFlywheelTime = System.nanoTime() / 1e9;

    public double flywheelFFTele(DcMotorEx m1, DcMotorEx m2, double target){
//
//        pidf.setPIDF(fP,fI,fD,fF);
        double curr = (m1.getVelocity()+m2.getVelocity())/2;
        double power = Values.flywheel_Values.fV*target + Values.flywheel_Values.fS +Values.flywheel_Values.flywheelPIDController.calculate(curr,target);
        m1.setPower(power);
        m2.setPower(power);
        return curr;
    }


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
            return pose.distanceFrom(Values.blueGoal);
        }else{
            return pose.distanceFrom(Values.redGoal);
        }
    }


    public void manualRelocalize(Follower follower){
        double y = follower.getPose().getY();
        if (Values.team==Values.Team.BLUE) {
            if (y>50){
                follower.setPose(new Pose(16.2,77.5,Math.toRadians(180)));
            }else {
                follower.setPose(new Pose(135 - 23.5, 6.5, Math.toRadians(0)));
            }
        }else{
            if (y>50){
                follower.setPose(new Pose(128,77.5,Math.toRadians(0)));
            }else {
                follower.setPose(new Pose(9+23.5,6.5,Math.toRadians(180)));
            }

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

    public static boolean inZone(Pose pose){
        double x = pose.getX();
        double y = pose.getY();
        boolean close = (y<=144) && (y>=x) && (y>=-x+144);
        boolean far = (y>=0) && (y<=x-48) && (y<-x+96);
        return close||far;
    }

    public double llAngle (Pose pose){
        double x = pose.getX();
        double y = pose.getY();

        double alpha = Math.toDegrees(Math.atan2(x-19.4,133.8-y));
        double lemma = Math.toDegrees(Math.atan2(137.3-y,x-12.5));
        return 90-alpha-lemma;
    }

    public static Pose sotm(Follower f,Pose target){
        Vector vel = f.getVelocity();
        Pose predicted = new Pose(
                target.getX()-vel.getXComponent()*weight, //tune weight
                target.getY()-vel.getYComponent()*weight
        );

        return predicted;
    }
    public double moveScale = 1;
    public double AutoAim(Follower f, Limelight3A ll){
        Pose botPose = f.getPose();
        Pose targetPose;
        if (Values.team==Values.Team.BLUE){
            targetPose = Values.blueGoal;
        }else{
            targetPose = Values.redGoal;
        }
//        targetPose = sotm(f,targetPose);
        double dx = botPose.getX()-targetPose.getX();
        double dy = targetPose.getY()-botPose.getY();
        double alpha = 180
                    - Math.toDegrees(botPose.getHeading())
                    - Math.toDegrees(Math.atan2(dy,dx));
        LLResult result= ll.getLatestResult();
        Values.txRaw = result.getTx();
        Values.tx = result.getTx();
        double angle = llAngle(botPose);
        double offsetAmt = LLOffset(angle);
        offsetAmt *= (Values.team == Values.Team.BLUE) ?1:-1;
        if (!result.isValid()){
            Values.tx=0;
        }else {
            Values.tx = result.getTx() - offsetAmt;
        }
        filteredX += test*Values.tx;
        double target =filteredX+alpha * 1725/18;
        target += Values.turretOverride;

        double wrapped = (target+17000)%34000;
        if (wrapped<0) wrapped+=34000;
        wrapped-=17000;
        return wrapped;
    }


//    public double AutoAim(Pose botPose, Limelight3A ll) {
//        double dx, dy, alpha;
//
//        if (Values.team == Values.Team.BLUE) {
//            dx = botPose.getX() - Values.blueGoal.getX();
//            dy = Values.blueGoal.getY() - botPose.getY();
//            alpha = 180
//                    - Math.toDegrees(botPose.getHeading())
//                    - Math.toDegrees(Math.atan2(dy, dx));
//        } else {
//            dx = botPose.getX() - Values.redGoal.getX();
//            dy = Values.redGoal.getY() - botPose.getY();
//            alpha = 180
//                    - Math.toDegrees(botPose.getHeading())
//                    - Math.toDegrees(Math.atan2(dy, dx));
//        }
//        LLResult result= ll.getLatestResult();
//        double dist = getDist(botPose);
//        if (dist > 120) {
//            offsetAmt = (Values.team == Values.Team.RED) ? -1 : 3;
//        }else{
//            offsetAmt = 0;
//        }
//        if (!result.isValid()){
//            Values.tx=0;
//        }else {
//            Values.tx = result.getTx() - offsetAmt;
//        }
//        filteredX+=Values.tx*test;
//        double target =filteredX+alpha * 1725/18;
//
//
//        double wrapped = (target+17000)%34000;
//        if (wrapped<0) wrapped+=34000;
//        wrapped-=17000;
//        return wrapped;
//    }

//    public double limelightOffset(Pose botPose){
//        double x = botPose.getX();
//        double y = botPose.getY();
//        if (Values.team == Values.Team.BLUE) {
//            double alpha = Math.toDegrees(Math.atan2(x - Values.blueTag.getX(), Values.blueTag.getY() - y));
//            double lambda = Math.toDegrees(Math.atan2(Values.blueGoal.getY() - y, x - Values.blueGoal.getX()));
//            return 90-alpha-lambda;
//        }else{
//            double alpha = Math.toDegrees(Math.atan2(Values.redTag.getX()-x,Values.redTag.getY()-y));
//            double lambda = Math.toDegrees(Math.atan2(Values.redGoal.getY()-y,Values.redGoal.getX()-x));
//            return 90-alpha-lambda;
//        }
//
//    }


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

    public double LLOffset(double angle){

        if (angle <= Values.llLUT.firstKey()) return Values.llLUT.firstEntry().getValue();
        if (angle >= Values.llLUT.lastKey())  return Values.llLUT.lastEntry().getValue();

        Map.Entry<Double, Double> lower = Values.llLUT.floorEntry(angle);
        Map.Entry<Double, Double> upper = Values.llLUT.ceilingEntry(angle);

        if (lower == null || upper == null) {
            return Values.llLUT.lastEntry().getValue();
        }

        if (lower.getKey().equals(upper.getKey())) {
            return lower.getValue();
        }

        double x0 = lower.getKey();
        double y0 = lower.getValue();
        double x1 = upper.getKey();
        double y1 = upper.getValue();

        double t = (angle - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
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
    public boolean getBeamRaw(DigitalChannel breakBeam1,DigitalChannel breakBeam2){
        if (!breakBeam1.getState() || !breakBeam2.getState()) {
            return true;
        }else{
            return false;
        }
    }

    private boolean intakeLock = false;
    private boolean outtakeLock = false;

    // if ball = 3 intake off

    public void countBalls(DigitalChannel breakBeam1,DigitalChannel breakBeam2,DigitalChannel breakBeam3,DigitalChannel breakBeam4) {
        //states
        boolean currentintake = getBeamRaw(breakBeam3,breakBeam4);
        boolean currentoutake = getBeamRaw(breakBeam1,breakBeam2);
        Values.bottomBlocked=currentintake;
        Values.topBlocked=currentoutake;
        //frame counter

        if (currentintake){
            Values.frameCountBlocked++;
            Values.frameCountUnblocked=0;
        }else{
            Values.frameCountBlocked=0;
            Values.frameCountUnblocked++;
        }

        if (currentoutake){
            Values.frameCountBlockedTop++;
            Values.frameCountUnblockedTop=0;
        }else{
            Values.frameCountBlockedTop=0;
            Values.frameCountUnblockedTop++;
        }

        //increment when enters
        if (currentintake && !last1){
            Values.frameCountBlocked=0;
            Values.counter++;
        }

        //decrement when leave
        if (!currentoutake && last2){
            Values.counter--;
        }
        //hard count to 3
        if (currentintake&& currentoutake&&Values.frameCountBlocked>15 && Values.frameCountBlockedTop>10 && Values.mode==Values.Modes.INTAKING){
            Values.counter=3;
        }
//        if (Values.counter==3 && !currentintake && currentoutake&&Values.frameCountBlocked>10 && Values.frameCountBlockedTop>10 && Values.mode==Values.Modes.INTAKING){
//            Values.counter=2;
//        }
        if (!currentintake && !currentoutake && Values.frameCountUnblocked>10 && Values.frameCountUnblockedTop>10){
            Values.counter=0;
        }
        if (Values.counter==3){
            if (Values.frameCountUnblocked>10 || Values.frameCountUnblockedTop>10){
                Values.counter=2;
            }
        }
        //if tree and 2 != tre count = 3
        //reset cont after shooting

        //clamp
        Values.counter = Math.min(3,Math.max(Values.counter,0));
        last1 = currentintake;
        last2 = currentoutake;
    }
    public double rpmComp(double dist) {

        if (dist <= Values.rpmLUT.firstKey()) return Values.rpmLUT.firstEntry().getValue();
        if (dist >= Values.rpmLUT.lastKey())  return Values.rpmLUT.lastEntry().getValue();

        Map.Entry<Double, Double> lower = Values.rpmLUT.floorEntry(dist);
        Map.Entry<Double, Double> upper = Values.rpmLUT.ceilingEntry(dist);

        if (lower == null || upper == null) {
            return Values.rpmLUT.lastEntry().getValue();
        }

        if (lower.getKey().equals(upper.getKey())) {
            return lower.getValue();
        }

        double x0 = lower.getKey();
        double y0 = lower.getValue();
        double x1 = upper.getKey();
        double y1 = upper.getValue();

        double t = (dist - x0) / (x1 - x0);
        double newtarget = y0 + t * (y1 - y0);
        return newtarget;
    }

    public String getstates(DigitalChannel breakBeam1,DigitalChannel breakBeam2,DigitalChannel breakBeam3,DigitalChannel breakBeam4){
        boolean state3 = !breakBeam1.getState(); //top in
        boolean state4 = !breakBeam2.getState(); //top out
        boolean state1 = !breakBeam3.getState(); //bottom
        boolean state2 = !breakBeam4.getState(); //bottom  down

        return String.format("1: %b 2: %b 3: %b 4: %b",state1,state2,state3,state4);
    }

    public void closeRapid (double dist) {
        if (Values.oldcounter < Values.counter) {
            Values.flywheel_Values.flywheelTarget = rpmComp(dist);
        }
        Values.oldcounter = Values.counter;
    }

    public void farslow(Double flywheelVel1, Double flywheelVel2,double dist, double time){
        double rpmError = Math.abs((flywheelVel1+flywheelVel2)/2 - Values.flywheel_Values.flywheelTarget);
        boolean atSpeed = rpmError < 200;
        if (atSpeed && time > 0.15) {
            hardware.intake.setPower(1);
            hardware.transfer.setPower(1);

        }else if (dist>110){
            hardware.intake.setPower(1);
            hardware.transfer.setPower(0);
        }
    }

}
/// turret servo angle LUT
/// ll: relocalization
///⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣤⣤⣤⣤⣤⣤⣤⣤⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀
/// ⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣿⡿⠛⠉⠙⠛⠛⠛⠛⠻⢿⣿⣷⣤⡀⠀⠀⠀⠀⠀
/// ⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⠋⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⠈⢻⣿⣿⡄⠀⠀⠀⠀
/// ⠀⠀⠀⠀⠀⠀⠀⣸⣿⡏⠀⠀⠀⣠⣶⣾⣿⣿⣿⠿⠿⠿⢿⣿⣿⣿⣄⠀⠀⠀
/// ⠀⠀⠀⠀⠀⠀⠀⣿⣿⠁⠀⠀⢰⣿⣿⣯⠁⠀⠀⠀⠀⠀⠀⠀⠈⠙⢿⣷⡄⠀
/// ⠀⠀⣀⣤⣴⣶⣶⣿⡟⠀⠀⠀⢸⣿⣿⣿⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣷⠀
/// ⠀⢰⣿⡟⠋⠉⣹⣿⡇⠀⠀⠀⠘⣿⣿⣿⣿⣷⣦⣤⣤⣤⣶⣶⣶⣶⣿⣿⣿⠀
/// ⠀⢸⣿⡇⠀⠀⣿⣿⡇⠀⠀⠀⠀⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠃⠀
/// ⠀⣸⣿⡇⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠉⠻⠿⣿⣿⣿⣿⡿⠿⠿⠛⢻⣿⡇⠀⠀
/// ⠀⣿⣿⠁⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣧⠀⠀
/// ⠀⣿⣿⠀⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⠀⠀
/// ⠀⣿⣿⠀⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⠀⠀
/// ⠀⢿⣿⡆⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⡇⠀⠀
/// ⠀⠸⣿⣧⡀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⠃⠀⠀
/// ⠀⠀⠛⢿⣿⣿⣿⣿⣇⠀⠀⠀⠀⠀⣰⣿⣿⣷⣶⣶⣶⣶⠶⠀⢠⣿⣿⠀⠀⠀
/// ⠀⠀⠀⠀⠀⠀⠀⣿⣿⠀⠀⠀⠀⠀⣿⣿⡇⠀⣽⣿⡏⠁⠀⠀⢸⣿⡇⠀⠀⠀
/// ⠀⠀⠀⠀⠀⠀⠀⣿⣿⠀⠀⠀⠀⠀⣿⣿⡇⠀⢹⣿⡆⠀⠀⠀⣸⣿⠇⠀⠀⠀
/// ⠀⠀⠀⠀⠀⠀⠀⢿⣿⣦⣄⣀⣠⣴⣿⣿⠁⠀⠈⠻⣿⣿⣿⣿⡿⠏⠀⠀⠀⠀
/// ⠀⠀⠀⠀⠀⠀⠀⠈⠛⠻⠿⠿⠿⠿⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀