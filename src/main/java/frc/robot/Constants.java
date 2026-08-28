package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final int ENCODER_COUNTS_PER_REVOLUTION = 312;
    public static final boolean ENCODER_INVERTED = true;

    //Change these for later when tuning
    public static final double kP= 0.1; 
    public static final double kI= 0.0;
    public static final double kD= 0.0;
    public static final double kS= 0.1;
    public static final double kV= 0.1;
    public static final double kA= 0.0;

    public static final double cruiseVelocity= 750;
    public static final double acceleration= 100; //RPM/s
    public static final double allowedProfileError= 5.0; //RPM

    public static final double MOI= 0.01;
    public static final double DriveGearRatio= 1.0; //might need to change
    public static final double wheelRadius=0.048; //definitely need to change

    //Change these later
    public static final Translation2d FrontLeft= new Translation2d(0.381, 0.381);
    public static final Translation2d FrontRight= new Translation2d(0.381, -0.381);
    public static final Translation2d BackLeft= new Translation2d(-0.381, 0.381);
    public static final Translation2d BackRight= new Translation2d(-0.381,-0.381);  

    public static final double maxLinearSpeedMetersPerSecond= 1.0;


    public static final double ARM_kP= 0.1; 
    public static final double ARM_kI= 0.0;
    public static final double ARM_kD= 0.0;
    public static final double ARM_kS= 0.1;
    public static final double ARM_kV= 0.1;
    public static final double ARM_kA= 0.0;

    public static final double ARM_cruiseVelocity= 750.0;
    public static final double ARM_acceleration= 100.0; //RPM/s
    public static final double ARM_allowedProfileError= 5.0; //RPM

    public static final double ARM_MOVING_VOLTAGE = 11.0;
    public static final double ARM_HOLDING_VOLTAGE = 2.0;
    public static final double ARM_HOLDING_THRESHOLD = 8.0;
    public static final double ARM_DONE_THRESHOLD = 2.0;

    public static final double ARM_OFFSET = -38.0;
    public static final double WRIST_PHYSTICAL_OFFSET = -300.0;
    public static final double WRIST_OFFSET = -90.0;

    public static final double ARM_MAX_POWER_DEGS = 8.0;

    public enum POSITIONS {
        STOWED(ARM_OFFSET, -WRIST_OFFSET),
        GROUND_COLECT(-25, -70.0),
        WAIT_COLLECT(-5.0, 6.0),
        COLLECT(13.0, 6.0), 
        L1(13.0, -2.0),
        L1_MID(5.0, 0.0),
        L1_PLACED(5.0, -0.0),
        L2(66.0, -45.0),
        L2_MID(60.0, -40.0),
        L2_PLACED(40.0, -40.0),
        L3(113.0, -65.0), 
        L3_MID(110.0, -60.0),
        L3_PLACED(85.0, -60.0);
        

        public final double armPos;
        public final double wristPos;

        POSITIONS(double armPos, double wristPos) {
            this.armPos = armPos;
            this.wristPos = wristPos;
        }
    }
}
