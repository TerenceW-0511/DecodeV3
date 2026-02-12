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
        public static PIDFController flywheelPIDController = new PIDFController(0, 0, 0, 0);
        public static double fP = 0.002;
        public static double fI = 0;
        public static double fD = 0.00002;

        public static double fF = 0.000376;

        public static double flywheelTarget=1500;
        public static double flywheelIdle = 1000; //MAX 2300

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
    public static final double LIMITER_OPEN=1,LIMITER_CLOSE=0.4;
    public static final double KICKER_DOWN = 0.35, KICKER_UP = 0.9;
    public static double turretPos=0.5,lastTurret = 0.5;
    public static boolean turretDeadSpot = false;
    public static double tx = 0;
    public static double hoodPos = 0.5;
    public static double autonFollowerX=135,autonFollowerY=6.5,autonHeading = 180,autonTurret=0;
    public static Pose blueGoal = new Pose(12.5,137.3);
    public static Pose redGoal = new Pose(131.5,137.3);
    public enum Team {
        RED,
        BLUE
    }

    public enum Modes {
        INTAKING,
        SHOOTING,

    }
    public static final TreeMap<Double,Double> hoodLUT = new TreeMap<>(
            //distance, hood
            Map.ofEntries(
                    Map.entry(35.2,0.46),
                    Map.entry(60.9,0.24),
                    Map.entry(84.7,0.12),
                    Map.entry(108.5,0.0)
            )
    );


    public static final TreeMap<Double, Double> turretDegreeToServoLUT = new TreeMap<>(
            Map.ofEntries(
//                    Map.entry(-171.0,0.0),
                    Map.entry(-150.5559498,0.0633),
                    Map.entry(-139.4296703,0.0954),
                    Map.entry(-90.0,0.2324),
                    Map.entry(-42.9324695,0.3672),
                    Map.entry(-38.7041722,0.401),
                    Map.entry(0.0,0.5),
                    Map.entry(38.7041722,0.6102),
                    Map.entry(90.0,0.7417),
                    Map.entry(139.4296703,0.9127),
                    Map.entry(150.5559498,0.9467)
//                    Map.entry(172.0,1.0)

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