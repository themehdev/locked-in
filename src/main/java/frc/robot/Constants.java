package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final int ENCODER_COUNTS_PER_REVOLUTION = 2150;
    public static final boolean ENCODER_INVERTED = false;

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

    public static final double ARM_MOVING_VOLTAGE = 10.0;
    public static final double ARM_HOLDING_VOLTAGE = 3.0;
    public static final double ARM_HOLDING_THRESHOLD = 8.0;
    public static final double ARM_DONE_THRESHOLD = 1.0;
}
