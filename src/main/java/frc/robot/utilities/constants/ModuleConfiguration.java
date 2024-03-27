package frc.robot.utilities.constants;

import java.util.Objects;

/**
 * A swerve module configuration.
 * <p>
 * A configuration represents a unique mechanical configuration of a module. For example, the Swerve Drive Specialties
 * Mk3 swerve module has two configurations, standard and fast, and therefore should have two configurations
 * ({@link SdsModuleConfigurations#MK3_STANDARD} and {@link SdsModuleConfigurations#MK3_FAST} respectively).
 */
public class ModuleConfiguration {
    private final double driveReduction;
    private final double steerReduction;

    /**
     * Creates a new module configuration.
     *
     * @param driveReduction The overall drive reduction of the module. Multiplying motor rotations by this value
     *                       should result in wheel rotations.
     * @param steerReduction The overall steer reduction of the module. Multiplying motor rotations by this value
     *                       should result in rotations of the steering pulley.
     */
    public ModuleConfiguration(double driveReduction, double steerReduction) {
        this.driveReduction = driveReduction;
        this.steerReduction = steerReduction;
    }

    /**
     * Gets the overall reduction of the drive system.
     * <p>
     * If this value is multiplied by drive motor rotations the result would be drive wheel rotations.
     */
    public double getDriveReduction() {
        return driveReduction;
    }

    /**
     * Gets the overall reduction of the steer system.
     * <p>
     * If this value is multiplied by steering motor rotations the result would be steering pulley rotations.
     */
    public double getSteerReduction() {
        return steerReduction;
    }

    @Override
    public int hashCode() {
        return Objects.hash(
                getDriveReduction(),
                getSteerReduction()
        );
    }

    @Override
    public String toString() {
        return "ModuleConfiguration{" + "driveReduction=" + driveReduction +", steerReduction=" + steerReduction +
                '}';
    }
}
