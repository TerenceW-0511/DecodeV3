package org.firstinspires.ftc.teamcode.pedroPathing;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.geometry.Pose;

import java.util.Map;
import java.util.TreeMap;

public class Values {
    public static Modes mode = Modes.INTAKING;
    public static Team team = Team.BLUE;
    public static boolean init = true;
    public static double turretOverride = 0;
    public static double llOverride = 0;

    public static class flywheel_Values {
//        public static double fP = 0.0045;
//        public static double fI = 0;
//        public static double fD = 0.00001;
//
//        public static double fF = 0.00038;
        public static double fP = 0.003;
        public static double fI = 0;
        public static double fD = 0.0000001;
        public static double fF = 0;
        public static double fV = 0.00036;
        public static double fS = 0.14;
        public static PIDFController flywheelPIDController = new PIDFController(fP, fI, fD, fF);

        public static double kV = 0;
        public static double kS = 0;


        public static double flywheelTarget=1800;
        public static double flywheelIdle = 1500; //MAX 2300

    }
    public static class transfer_Values {
        public static PIDFController transferPIDController = new PIDFController(0, 0, 0, 0);

        public static double trP = 0.0004;
        public static double trI = 0;
        public static double trD = 0.00001;
        public static double trF = 0.00045;

        public static double transferTarget=0;
        public static double transferUp = 3000,transferIntake = 500;
    }

    public static class intake_Values {
        public static PIDFController intakePIDController = new PIDFController(0, 0, 0, 0);
        public static double iP = 0;
        public static double iI = 0;
        public static double iD = 0.000001;

        public static double iK = 0.00048;
        public static double intakeShoot = 3000;
        public static double intakeIntaking=1500;
        public static double intakeTarget=0;
        public static double intakeHold = 1000;
    }

    public static class turret_Values {
        public static PIDFController turretPIDController = new PIDFController(0,0,0,0);
        public static double kP = 0.0002;
        public static double kI = 0.0;
        public static double kD = 0.000005;
        public static final double MAX = 17500, MIN = -17500;// negative is clockwise
        public static double idle = 0.5;
    }

    public static final double TURRET_RIGHT = 0;
    public static final double LIMITER_OPEN=0.9,LIMITER_CLOSE=0.35;
    public static double turretPos=0.5,lastTurret = 0.5;
    public static boolean turretDeadSpot = false;
    public static double tx = 0, llOffset = 0;
    public static double hoodPos = 0.5;
    public static double autonFollowerX=9,autonFollowerY=8,autonHeading = 0,autonTurret=0;
    public static Pose blueGoal = new Pose(12.5,137.3);
    public static Pose blueTag = new Pose(19.4,133.8);
    public static Pose redGoal = new Pose(131.5,137.3);
    public static Pose redTag = new Pose(124.6,133.8);
    public enum Team {
        RED,
        BLUE
    }

    public enum Modes {
        INTAKING,
        SHOOTING,

    }
    public static final TreeMap<Double,Double> hoodLUT = new TreeMap<>(
            //distance, hood, k
            Map.ofEntries(

                    Map.entry(26.7,1.0),
                    Map.entry(29.1,1.0),
                    Map.entry(57.0,0.64),
                    Map.entry(60.8,0.48),
                    Map.entry(96.0,0.48),
                    Map.entry(120.0,0.48),
                    Map.entry(134.75,0.2),
                    Map.entry(157.8,0.0)

            )
    );


//
//    public static final TreeMap<Double, Double> llLUT = new TreeMap<>(
//            Map.ofEntries(
//                    Map.entry()
//            )
//    );
    public static void reset(){

        turretPos = 0.5;
        llOverride = 0;
    }
    //TODO: retune flywheel pid, do auton turret? should be the only thing that changed
}