package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AprilTagAimCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;
import static frc.robot.Utilities.*;

import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {    
    // The robot's subsystems are defined here
    public static final Drivetrain drivetrain = new Drivetrain();

    // The drive team controllers are defined here
    public static final XboxController driverController = new XboxController(0);
    // public static final XboxController operatorController = new XboxController(1);

    // Limits maximum speed
    private double maxSpeedFactor = .2;

    public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up the default command for the drivetrain
        // The controls are for field-oriented driving
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(drivetrain,
                () -> -modifyAxis(driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
                () -> -modifyAxis(driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
                () -> -modifyAxis(driverController.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                        * maxSpeedFactor));

        // Configure the button bindings
        configureButtonBindings();
        configureAutos();
    }

    /** Button -> command mappings are defined here. */
    private void configureButtonBindings() {
        // Driver Controller Bindings
        // Pressing A button zeros the gyroscope
        new Trigger(driverController::getAButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyroscope()));
        // Pressing Y button locks the wheels in an X pattern
        new Trigger(driverController::getYButton).onTrue(new InstantCommand(() -> drivetrain.toggleWheelsLocked()));
        // Pressing B button rotates bot to face apriltag
        new Trigger(driverController::getBButton).onTrue(new AprilTagAimCommand(drivetrain,
                "speaker",
                () -> -modifyAxis(driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor, 
                () -> -modifyAxis(driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor));
        // Pressing the Right Bumper shifts to high speed
        new Trigger(driverController::getRightBumper).onTrue(new InstantCommand(() -> speedUp()));
        // Pressing the Left Bumper shifts to low speed
        new Trigger(driverController::getLeftBumper).onTrue(new InstantCommand(() -> speedDown()));
    }

    public void configureAutos() {
        //autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    

    /**
     * This method returns the autonomous routine that will be run at the start of
     * the match.
     * <p>
     * For now, we only have one routine, so it just returns that one.
     * Once we have more than one routine, we will want to implement a chooser
     * dropdown on the dashboard.
     * 
     * @return The autonomous routine that will be run
     */
    public Command getAutonomousCommand() {
        //return autoChooser.getSelected();
        return new PathPlannerAuto("New Auto");
    }

    public void speedUp() {
        maxSpeedFactor += .1;
        maxSpeedFactor = MathUtil.clamp(maxSpeedFactor, .1, 1.);
    }

    public void speedDown() {
        maxSpeedFactor -= .1;
        maxSpeedFactor = MathUtil.clamp(maxSpeedFactor, .1, 1.);
    }
}
