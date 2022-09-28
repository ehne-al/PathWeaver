package frc.robot;


public class Constants {


    public final class Drivetrain{

        //Talox SRX Specifications
        public static final int CPR = -4096;

        //Robot Characteristics
        public static final double TRACK_WIDTH = 0.381 * 2;
        public static final double WHEEL_RADIUS = 0.0508;

        public static final double MAX_SPEED_METERSECOND = 2;
        public static final double MAX_ACCELERATION_METERSECONDSQUARED = 1;

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
        public static final int RIGHT_ENCODER_B_ID = 4;

        //PID
        public static final double KP = 0.8;
        public static final double KI = 0;
        public static final double KD = 0;

        //Ramsette
        public static final double RAMSETTE_B = 2;
        public static final double RAMSETTE_ZETA = 0.7;






    }
}
