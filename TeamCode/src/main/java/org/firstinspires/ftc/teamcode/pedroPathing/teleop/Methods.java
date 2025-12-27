package org.firstinspires.ftc.teamcode.pedroPathing.teleop;
import com.arcrobotics.ftclib.controller.PIDFController;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Methods {
    private boolean firstLoop = true;
    private double lastPos;
    private double lastTime;
    private double lastTarget=0;
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


    public void Relocalize(){

    }
    public void Transfer(){
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
        servoAngle = Math.min(1,Math.max(0,servoAngle));
        return 1-servoAngle;

    }

    public double hoodControl(Follower follower, DcMotorEx flywheel1, DcMotorEx flywheel2){
        // 3d graph equation
        double vel = (flywheel1.getVelocity() + flywheel2.getVelocity())/2;
        double dist = getDist(follower);
        double numerator = (-(c + f*dist) + Math.sqrt((c + f*dist)*(c + f*dist) - 4*e*(a + b*dist + d*dist*dist - vel))) / (2*e);
        numerator = Math.min(1,Math.max(0,numerator));
        return numerator;


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