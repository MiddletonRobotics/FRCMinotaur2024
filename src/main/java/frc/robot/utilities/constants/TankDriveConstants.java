package frc.robot.utilities.constants;

public class TankDriveConstants {
    public final int frontLeftID;
    public final int frontRightID;
    public final int backLeftID;
    public final int backRightID;

    /**
    * @param frontLeftID
    * @param frontRightID
    * @param backLeftID
    * @param backRightID
    */

    public TankDriveConstants(int frontLeftID, int frontRightID, int backLeftID, int backRightID) {
        this.frontLeftID = frontLeftID;
        this.frontRightID = frontRightID;
        this.backLeftID = backLeftID;
        this.backRightID = backRightID;
    }
}
