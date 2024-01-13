package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
    // NavX connected over MXP
    public final AHRS navx;

    /**
     * For user to reset zero for "forward" on the robot while maintaining absolute
     * field zero for odometry
     */
    private double gyroOffset;

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    // The speed of the robot in x and y translational velocities and rotational velocity
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Boolean statement to control locking the wheels in an X-position
    private boolean wheelsLocked = false;

    private final SwerveDrivePoseEstimator poseEstimator;

    public Drivetrain() {
        // Initialize all modules
        backRightModule = new SwerveModule(
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET);

        backLeftModule = new SwerveModule(
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET);

        frontRightModule = new SwerveModule(
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET);

        frontLeftModule = new SwerveModule(
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET);

        // Zero all relative encoders
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();

        // Initialize and zero gyro
        navx = new AHRS(SPI.Port.kMXP);
        zeroGyroscope();

        poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getGyroscopeRotation(), getSwerveModulePositions(),
                new Pose2d());
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        gyroOffset = -getGyroscopeAngle();
    }

    public double getGyroOffset() {
        return gyroOffset;
    }

    /**
     * Toggles whether or not the wheels are locked. If they are, the wheels are
     * crossed into an X pattern and any other drive input has no effect.
     */
    public void toggleWheelsLocked() {
        wheelsLocked = !wheelsLocked;
    }

    public double getGyroscopeAngle() {
        return Math.IEEEremainder(360. - navx.getAngle(), 360.);
    }

    // Returns the measurment of the gyroscope yaw. Used for field-relative drive
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getGyroscopeAngle());
    }

    /** @return Pitch in degrees, -180 to 180 */
    public double getPitch() {
        return navx.getPitch();
    }

    /** @return Roll in degrees, -180 to 180 */
    public double getRoll() {
        return navx.getRoll();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), pose);
    }

    public void updateOdometry() {
        poseEstimator.update(getGyroscopeRotation(), getSwerveModulePositions());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        backLeftModule.setState(states[2]);
        backRightModule.setState(states[3]);
    }

    /**
     * Used to drive the robot with the provided ChassisSpeed object. However, if
     * the robot is in autobalance mode, the ChassisSpeed object is ignored, and a
     * new one is calculated based off the pitch and roll of the robot.
     * 
     * @param chassisSpeeds The translational and rotational velocities at which to
     *                      drive the robot.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * This method is used to command the individual module states based off the
     * ChassisSpeeds object
     */
    @Override
    public void periodic() {
        // Convert from ChassisSpeeds to SwerveModuleStates
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        // Make sure no modules are being commanded to velocites greater than the max
        // possible velocity
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        if (!wheelsLocked) {
            // If we are not in wheel's locked mode, set the states normally
            frontLeftModule.setState(states[0]);
            frontRightModule.setState(states[1]);
            backLeftModule.setState(states[2]);
            backRightModule.setState(states[3]);
        } else {
            // If we are in wheel's locked mode, set the drive velocity to 0 so there is no
            // movment, and command the steer angle to either plus or minus 45 degrees to
            // form an X pattern.
            frontLeftModule.lockModule(45);
            frontRightModule.lockModule(-45);
            backLeftModule.lockModule(-45);
            backRightModule.lockModule(45);
        }

        // Update the odometry
        updateOdometry();

        // Diagnostics
        SmartDashboard.putNumber("Front Left Absolute", frontLeftModule.getAbsolutePosition());
        SmartDashboard.putNumber("Front Right Absolute", frontRightModule.getAbsolutePosition());
        SmartDashboard.putNumber("Back Left Absolute", backLeftModule.getAbsolutePosition());
        SmartDashboard.putNumber("Back Right Absolute", backRightModule.getAbsolutePosition());
        SmartDashboard.putNumber("Gyro", getGyroscopeAngle());
    }
}
