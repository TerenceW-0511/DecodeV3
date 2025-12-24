package org.firstinspires.ftc.teamcode.pedroPathing.teleop;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

public class Values {
    public static Modes mode = Modes.INTAKING;
    public static Team team = Team.RED;
    public static  double HOOD_FAR = 0;
    public static  double HOOD_CLOSE = 0;

    public static class flywheel_Values {
        public static PIDFController flywheelPIDController = new PIDFController(0, 0, 0, 0);
        public static double fP = 0.0011;
        public static double fI = 0;
        public static double fD = 0.0000001;

        public static double fF = 0.0004;
        public static double flywheelTarget=0;
        public static double flywheelVelocity = 2000; //MAX 2300

    }
    public static class transfer_Values {
        public static PIDFController transferPIDController = new PIDFController(0, 0, 0, 0);

        public static double trP = 0.0004;
        public static double trI = 0;
        public static double trD = 0.00001;
        public static double trF = 0.00045;

        public static double transferTarget=0;
        public static double transferUp = 2000,transferIntake = 1000;
    }

    public static class intake_Values {
        public static PIDFController intakePIDController = new PIDFController(0, 0, 0, 0);
        public static double iP = 0;
        public static double iI = 0;
        public static double iD = 0.000001;

        public static double iK = 0.00048;
        public static double intakeIntaking=1500;
        public static double intakeTarget=0;
    }

    public static final double TURRET_RIGHT = 0;
    public static final double LIMITER_OPEN=0.65,LIMITER_CLOSE=0.3;
    public static double turretPos=0.5;
    public static double hoodPos = 0.5;
    public enum Team {
        RED,
        BLUE
    }

    public enum Modes {
        INTAKING,
        SHOOTING,

    }
    public static void reset(){
        turretPos = 0.5;
    }
}