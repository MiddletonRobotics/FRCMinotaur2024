//just a test to make sure this all commits properly
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Utilities.Constants.Constants;
import frc.robot.Utilities.Drivers.MinoGamepad;
import frc.robot.Utilities.Section;
import com.revrobotics.CANSparkMax;


public class Intake extends Subsystem {

    private CANSparkMax topIntake;
    private CANSparkMax bottomIntake;
    private CANSparkMax rotateIntake;


    private static Intake instance = null;

    private Intake() {
        topIntake = new CANSparkMax​(62 intakeTopID, CANSparkLowLevel.MotorType.kBrushless);
        bottomIntake = new CANSparkMax(7 intakeBottomID, CANSparkLowLevel.MotorType.kBrushless);
        rotateIntake = new CANSparkMax(48 intakeRotateID, CANSparkLowLevel.MotorType.kBrushless); //ints are just there as placeholders till we get actual ones, same with ids
    }

//we felt a little silly with the names
    public void intakeConsume() {
        topIntake.set(0.5); //whatever makes motor take thingy
        bottomIntake.set(0.5); //make it match top intake(reverse probably if motors are backwards)
    }

    public void intakeRegurgitate() {
        topIntake.set(-0.5); //whatever makes motor release thingy
        bottomIntake.set(-0.5); //make it match bottom
    }

    public void intakeForward() {
        rotateIntake.set(0.5); //placeholder values for rn
    }

    public void intakeBackward() {
        rotateIntake.set(-0.5); //same comment as intakeForward
    }

    @Override
    public void reset() {
        topIntake.set(0);
        bottomIntake.set(0);
    }

}
