package frc.robot.utilities.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int swerveEncoderID;
    public final double angleOffset;

    /**
    * @param driveMotorID
    * @param angleMotorID
    * @param swerveEncoderID
    * @param angleOffset
    */

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int swerveEncoderID, double angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.swerveEncoderID = swerveEncoderID;
        this.angleOffset = angleOffset;
    }
}
