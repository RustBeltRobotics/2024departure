package frc.robot.commands;

import static frc.robot.Constants.rotation_P;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class AprilTagAim extends Command {
    double tx = LimelightHelpers.getTX("limelight");
    private final Drivetrain drivetrain;

    private double steeringAdjust; 

    public AprilTagAim(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        double headingError = tx;
        steeringAdjust = rotation_P * tx;
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            0,
            steeringAdjust,
            Rotation2d.fromDegrees(drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset())));
    //left_command+=steering_adjust;
    //right_command-=steering_adjust;
    }
    @Override
    public void end(boolean interrupted) {
    }
        
}
