/* (C) Robolancers 2024 */
package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class Alliance {
    public static boolean isRed() {
        Optional<DriverStation.Alliance> myAlliance = DriverStation.getAlliance();
        return myAlliance.isPresent() && myAlliance.get() == DriverStation.Alliance.Red;
    }
}