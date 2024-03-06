package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Utilities.Constants.Constants;
import frc.robot.Utilities.Drivers.MinoGamepad;
import frc.robot.Utilities.Section;
import com.revrobotics.CANSparkMax;


public class Intake extends Subsystem {

    private CANSparkMax gripIntake;
    private CANSparkMax rotateIntake;


    private static Intake instance = null;

    private Intake() {
        gripIntake = new CANSparkMax​(Constants.IntakeConstants.gripIntakeID, CANSparkLowLevel.MotorType.kBrushless);
        rotateIntake = new CANSparkMax(Constants.IntakeConstants.rotateIntakeID, CANSparkLowLevel.MotorType.kBrushless); //ints are just there as placeholders till we get actual ones, same with ids
    }

//we felt a little silly with the names
    public void intakeConsume() {
        gripIntake.set(0.5); //whatever makes motor take thingy
    }

    public void intakeRegurgitate() {
        gripIntake.set(-0.5); //whatever makes motor release thingy
    }

    public void intakeForward() {
        rotateIntake.set(0.5); //placeholder values for rn
    }

    public void intakeBackward() {
        rotateIntake.set(-0.5); //same comment as intakeForward
    }

    @Override
    public void reset() {
        gripIntake.set(0);
    }

}
