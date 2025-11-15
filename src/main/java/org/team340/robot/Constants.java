package org.team340.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double kVoltage = 12.0;

    public static final int kDriver = 0;
    public static final int kCoDriver = 1;

    /**
     * The RobotMap class defines CAN IDs, CAN bus names, DIO/PWM/PH/PCM channel
     * IDs, and other relevant identifiers for addressing robot hardware.
     */
    public static final class RobotMap {

        public static final int kFlMove = 7;
        public static final int kFlTurn = 5;
        public static final int kFrMove = 8;
        public static final int kFrTurn = 9;
        public static final int kBlMove = 4;
        public static final int kBlTurn = 6;
        public static final int kBrMove = 3;
        public static final int kBrTurn = 2;

        public static final int kElevatorLead = 20;
        public static final int kElevatorFollow = 21;
        //not in 30
        public static final int kIntake = 30;

        public static final int kFlEncoder = 13;
        public static final int kFrEncoder = 12;
        public static final int kBlEncoder = 10;
        public static final int kBrEncoder = 11;
        // public static final int stopSwitch = 
    }
}
