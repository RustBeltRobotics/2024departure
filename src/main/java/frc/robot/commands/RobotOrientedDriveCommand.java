package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/**
 * This command is used to drive the robot with a coordinate system that is
 * relative to the robot, not the field.
 */
public class RobotOrientedDriveCommand extends Command {
    private final Drivetrain drivetrain;

    // DoubleSupplier objects need to be used, not double
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final IntSupplier povSupplier;

    GenericEntry robotOrientEntry = Shuffleboard.getTab("Competition")
            .add("Robot Oriented", false)
            .withWidget("Boolean Box")
            .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"))
            .getEntry();

    public RobotOrientedDriveCommand(Drivetrain drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            IntSupplier povSupplier) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.povSupplier = povSupplier;

        // Command requires the drivetrain subsystem
        addRequirements(drivetrain);
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * Send the input x, y, and rotation velocities to the drivetrain's drive method
     * as a ChassisSpeed object.
     */
    @Override
    public void execute() {
        robotOrientEntry.setBoolean(true);
        int pov = povSupplier.getAsInt();
        if (pov == -1) {
            drivetrain.drive(new ChassisSpeeds(
                    translationXSupplier.getAsDouble(),
                    translationYSupplier.getAsDouble(),
                    rotationSupplier.getAsDouble()));
        } else {
            drivetrain.drive(new ChassisSpeeds(
                    Math.cos(Math.toRadians(pov)),
                    Math.sin(Math.toRadians(pov - 180)),
                    0.));
        }
    }

    /** When the drive method is interupted, set all velocities to zero. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
        robotOrientEntry.setBoolean(false);
    }
}
