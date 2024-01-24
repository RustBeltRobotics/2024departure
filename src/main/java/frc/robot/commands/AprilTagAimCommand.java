package frc.robot.commands;

import static frc.robot.Constants.limelightName;
import static frc.robot.Constants.rotation_P;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;

public class AprilTagAimCommand extends Command {
    private double tx;
    private String target;
    private double sightedTID;
    private double targetTID;
    private double targetTID2 = -1;
    private double steeringAdjust; 
    private DoubleSupplier stickX;
    private DoubleSupplier stickY;
    private boolean finished;
    private Optional<Alliance> alliance = DriverStation.getAlliance();

    private final Drivetrain drivetrain;
    public AprilTagAimCommand(Drivetrain drivetrain, String target, DoubleSupplier stickX, DoubleSupplier stickY) {
        this.drivetrain = drivetrain;
        this.target = target;
        this.stickX = stickX;
        this.stickY = stickY;
        addRequirements(drivetrain);
    }
    @Override
    public void initialize() {
        finished = false;
    }
    @Override
    public void execute() {
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                switch (target) {
                case "speaker":
                    targetTID = 3;
                    targetTID2 = 4;
                    break;
                case "amp":
                    targetTID = 5;
                    break;
                case "source":
                    targetTID = 9;
                    targetTID2 = 10;                    
                    break;
                }
            }
        } else {//its blue
                switch (target) {
                case "speaker":
                    targetTID = 7;
                    targetTID2 = 8;
                    break;
                case "amp":
                    targetTID = 6;
                    break;
                case "source":
                    targetTID = 1;
                    targetTID2 = 2;
                    break;
                }
        }
        //determine if the primary tag id matches our target tag id
        sightedTID = LimelightHelpers.getFiducialID(limelightName);
        if (sightedTID == targetTID || sightedTID == targetTID2) {
            tx = LimelightHelpers.getTX(limelightName);
            steeringAdjust = rotation_P * tx;
            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                stickX.getAsDouble(),
                stickY.getAsDouble(),
                steeringAdjust,
                Rotation2d.fromDegrees(drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset())));
        }
        finished = true;
    }
    @Override
    public boolean isFinished() {
        if (finished == true){
            return true;
        } else {
            return false;
        }
    }
}