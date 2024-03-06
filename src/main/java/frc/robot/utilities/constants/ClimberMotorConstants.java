package frc.robot.utilities.constants;

//I am doing something very funky here but i will make it work pinky promise

public class ClimberMotorConstants {
    public final int climbMotorID;
    public final int climbEncoderID;

    /**
    * @param climbMotorID
    * @param climbEncoderID
    */

    public ClimberMotorConstants(int climbMotorID, int climbEncoderID) {
        this.climbMotorID = climbMotorID;
        this.climbEncoderID = climbEncoderID;
    }
}