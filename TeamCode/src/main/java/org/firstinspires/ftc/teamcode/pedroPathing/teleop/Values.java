package org.firstinspires.ftc.teamcode.pedroPathing.teleop;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

public class Values {
    public Modes mode = Modes.INTAKING;
    public static  double HOOD_FAR = 0;
    public static  double HOOD_CLOSE = 0;

    public static class flywheel_Values {
        public static PIDFController flywheelPIDController = new PIDFController(0, 0, 0, 0);
        public static double fP = 0;
        public static double fI = 0;
        public static double fD = 0;

        public static double fF = 0;
        public double flyWheelVelocity = 1500;
    }
    public static class transfer_Values {
        public static PIDFController transferPIDController = new PIDFController(0, 0, 0, 0);

        public static double trP = 0.001;
        public static double trI = 0;
        public static double trD = 0.0000001;
        public static double trF = 0.00036;


        public static double transferUp = 2000,transferIntake = 1400;
    }

    public static class intake_Values {
        public static PIDFController intakePIDController = new PIDFController(0, 0, 0, 0);
        public static double iP = 0;
        public static double iI = 0;
        public static double iD = 0.000001;

        public static double iK = 0.00048;
        public static double intakeTarget = 1500;
    }

    public static final double TURRET_RIGHT = 0;

    public enum Modes {
        INTAKING,
        SHOOTING,
        REST
    }
}