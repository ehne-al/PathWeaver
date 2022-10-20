package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constants {


    public final class Drivetrain{

        //Talox SRX Specifications
        public static final int CPR = -1000;

        //Robot Characteristics
        public static final double TRACK_WIDTH = 0.381 * 2;
        public static final double WHEEL_RADIUS = 0.0508;
        public static final double WHEEL_CIRCUNFERENCE = 0.3191;


        public static final double MAX_SPEED_METERSECOND = 0.4;
        public static final double MAX_ACCELERATION_METERSECONDSQUARED = 0.4;

        public static final double MAX_ANGULAR_SPEED = Math.PI;

        //Controllers

        public static final int LEFT_SLAVE_ID = 1;
        public static final int LEFT_MASTER_ID = 2;
        public static final int RIGHT_SLAVE_ID = 3;
        public static final int RIGHT_MASTER_ID = 4;

        //Encoder
        public static final int LEFT_ENCODER_A_ID = 0;
        public static final int LEFT_ENCODER_B_ID = 1;
        public static final int RIGHT_ENCODER_A_ID = 2;
        public static final int RIGHT_ENCODER_B_ID = 3;

        //PID
        public static final double KP = 0.5;
        public static final double KI = 0;
        public static final double KD = 0;

        //Ramsette
        public static final double RAMSETTE_B = 2;
        public static final double RAMSETTE_ZETA = 0.7;

    }

    //SUSSSS
    public static final double ksVolts = 0.022;
    public static final double kvVoltSecondsPerMeter = 0.098;
    public static final double kaVoltSecondsSquaredPerMeter = 0.02;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 0.2;

    public static final double kTrackwidthMeters = 0.069;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 0.05;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.05;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
