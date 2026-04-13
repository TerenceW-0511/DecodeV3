package org.firstinspires.ftc.teamcode.pedroPathing;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import java.util.Map;
import java.util.TreeMap;
@Config
@Configurable
public class Values {
    public static Modes mode = Modes.INTAKING;
    public static Team team = Team.BLUE;
    public static boolean init = true,rumble=false;
    public static double turretOverride = 0;
    public static double llOverride = 0;


    public static class flywheel_Values {
//        public static double fP = 0.0045;
//        public static double fI = 0;
//        public static double fD = 0.00001;
//
//        public static double fF = 0.00038;
        public static double fP = 0.0045;
        public static double fI = 0;
        public static double fD = 0.0000001;
        public static double fF = 0;
        public static double fV = 0.000364;
        public static double fS = 0.04;
        public static PIDFController flywheelPIDController = new PIDFController(fP, fI, fD, fF);

        public static double kV = 0;
        public static double kS = 0;


        public static double flywheelTarget=1000;
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
        public static double kP = 0.00019;
        public static double kI = 0.0;
        public static double kD = 0.000006;
        public static final double MAX = 17500, MIN = -17500;// negative is clockwise
        public static double idle = 0.5;

    }
    public static int oldcounter = 0;
    public static final double TURRET_RIGHT = 0;
    public static final double LIMITER_OPEN=1,LIMITER_CLOSE=0.5;
    public static double turretPos=0.5,lastTurret = 0.5;
    public static boolean turretDeadSpot = false;
    public static double offsetAngle=0,txRaw=0,tx = 0, llOffset = 0;
    public static double hoodPos = 0.5;
    public static double autonFollowerX=15.6,autonFollowerY=113.7,autonHeading = Math.toRadians(180),autonTurret=0;
    public static Pose blueGoal = new Pose(12.5,137.3);
    public static Pose redGoal = new Pose(131.5,137.3);

    public static boolean topBlocked=false,bottomBlocked=false;
    public static int counter = 0,frameCountBlocked = 0,frameCountBlockedTop = 0,frameCountUnblocked=0,frameCountUnblockedTop=0;
    public static double rDecay = 0, mDecay = 0,aMax = 0;
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

                    Map.entry(29.6,1.0),
                    Map.entry(46.6,0.62),
                    Map.entry(53.6,0.46),
                    Map.entry(77.4,0.24),
                    Map.entry(90.0,0.24)

            )
    );
    public static final TreeMap<Double,Double> rpmLUT = new TreeMap<>(
            Map.ofEntries(
                    Map.entry(40.12464708182086, 1470.0),
                    Map.entry(49.14520309175569,1500.0),
                    Map.entry(51.57541835973636,1580.0),
                    Map.entry(61.20058168254951,1690.0),
                    Map.entry(72.20668678918508, 1890.0),
                    Map.entry(85.50943734702756, 2150.0)
            )
    );

    public static final TreeMap<Double,Double> llLUT = new TreeMap<>(
        Map.ofEntries(
                Map.entry(-3.33,-6.6),
                Map.entry(-1.8,-4.0),
                Map.entry(-0.7,-1.3),
                Map.entry(0.2,0.4),
                Map.entry(1.3,1.8),
                Map.entry(2.3,2.6),
                Map.entry(3.4,7.8),
                Map.entry(4.8,8.2),
                Map.entry(5.3,10.1)
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
        Values.counter=0;
        Values.turretOverride=0;
    }
    //TODO: retune flywheel pid, do auton turret? should be the only thing that changed
}