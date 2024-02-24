package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonSRXConfigurator {

    public enum MotorType {
        k775Pro,
        kMiniCIM,
        kCIM,
        kBag,
        kRedline,
    }

    public void toggleInverted(TalonSRX talon) {
        boolean invertedState = talon.getInverted();

        if(invertedState) {
            talon.setInverted(false);
        } else {
            talon.setInverted(true);
        }
    }

    public void setNeutralMode(TalonSRX talon, NeutralMode mode) {
        talon.setNeutralMode(mode);
    }

    public void setControlMode(TalonSRX talon) {
        talon.enableCurrentLimit(true);
        talon.configPeakCurrentLimit(40);
    }

    public void setMotorType(TalonSRX talon, MotorType type) {
        switch(type) {
            case k775Pro:
                talon.configPeakCurrentLimit(100);
                talon.configContinuousCurrentLimit(60);
                break;
            case kMiniCIM:
                talon.configPeakCurrentLimit(100);
                talon.configContinuousCurrentLimit(60);
                break;
            case kCIM:
                talon.configPeakCurrentLimit(100);
                talon.configContinuousCurrentLimit(60);
                break;
            case kBag:
                talon.configPeakCurrentLimit(100);
                talon.configContinuousCurrentLimit(60);
                break;
            case kRedline:
                talon.configPeakCurrentLimit(67);
                talon.configContinuousCurrentLimit(60);
                break;
        }
    }
}
