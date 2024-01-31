package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** Maximum battery voltage */
    public static final double MAX_VOLTAGE = 12.;

    // Spark Max current limits
    /** Smart drive motor current */
    public static final int DRIVE_SMART_CURRENT_LIMIT = 30;
    /** Smart steer motor current */
    public static final int STEER_SMART_CURRENT_LIMIT = 40;
    /** Secondary drive motor current */
    public static final int DRIVE_SECONDARY_CURRENT_LIMIT = 80;
    /** Secondary steer motor current */
    public static final int STEER_SECONDARY_CURRENT_LIMIT = 80;
    /** Smart current limit applied to NEOs */
    public static final int NEO_SMART_CURRENT_LIMIT = 60;
    /** Secondary current limit applied to NEOs */
    public static final int NEO_SECONDARY_CURRENT_LIMIT = 80;
    /** Smart current limit applied to NEO 550s */
    public static final int NEO550_SMART_CURRENT_LIMIT = 20;
    /** Secondary current limit applied to NEO 550s */
    public static final int NEO550_SECONDARY_CURRENT_LIMIT = 30;

    // Drivetrain Constants
    // FIXME: These values for Arno and departure. CRESCENDO uses SDS MK4 L3
    // modules, mounted on a 24" x 24" frame perimeter
    /**
     * The left-to-right distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.47;

    // FIXME: These values for Arno and departure. CRESCENDO uses SDS MK4 L3
    // modules, mounted on a 24" x 24" frame perimeter
    /**
     * The front-to-back distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.47;

    public static final double DRIVETRAIN_BASE_RADIUS = Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS, DRIVETRAIN_WHEELBASE_METERS) / 2;

    /**
     * Creates a swerve kinematics object, to convert desired chassis velocity into
     * individual module states
     */
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2., DRIVETRAIN_WHEELBASE_METERS / 2.),
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2., -DRIVETRAIN_WHEELBASE_METERS / 2.),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2., DRIVETRAIN_WHEELBASE_METERS / 2.),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2., -DRIVETRAIN_WHEELBASE_METERS / 2.));

    // FIXME: These values for Arno and departure. CRESCENDO uses SDS MK4 L3 modules
    /** Conversion between rotations and meters */
    public static final double DRIVE_POSITION_CONVERSION = Math.PI
            * 0.10033 // SdsModuleConfigurations.MK4I_L2.getWheelDiameter()
            * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // SdsModuleConfigurations.MK4I_L2.getDriveReduction();

    /** Conversion between rotations per minute and meters per seconds */
    public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.;

    /**
     * The maximum linear velocity of the robot in meters per second. This is a
     * measure of how fast the robot can move linearly.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676. * DRIVE_VELOCITY_CONVERSION;

    public static final double MAX_VELOCITY_PRECISION_MODE_METERS_PER_SECOND = 0.5;

    /**
     * The maximum angular velocity of the robot in radians per second. This is a
     * measure of how fast the robot can rotate in place.
     */
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2., DRIVETRAIN_WHEELBASE_METERS / 2.);

    // FIXME: These values for Arno and departure. CRESCENDO uses SDS MK4 L3 modules
    /** Conversion between rotations and degrees */
    public static final double STEER_POSITION_CONVERSION = 360. * (14.0 / 50.0) * (10.0 / 60.0); // SdsModuleConfigurations.MK4I_L2.getSteerReduction();

    /** Conversion between rotations per minute and degrees per seconds */
    public static final double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / 60.;

    // FIXME: These values are probably a solid starting point, given MK4I's and
    // MK4's have a different steering ratio by about 40%
    // Steer PID Constants
    public static final double STEER_P = 0.008;
    public static final double STEER_I = 0.;
    public static final double STEER_D = 0.0002;

    /** Max velocity while following a trajectory. Meters per second */
    public static final double MAX_TRAJECTORY_VELOCITY = 3.;

    /**
     * Max acceleration while following a trajectory. Meters per second per second
     */
    public static final double MAX_TRAJECTORY_ACCELERATION = 2.;

    /** Max velocity while following balance auto trajectory. Meters per second */
    public static final double MAX_BALANCE_TRAJECTORY_VELOCITY = 3.;

    // PID Constants for translation and rotation moves
    public static final double translation_P = 10.0;
    public static final double translation_I = 0.0;
    public static final double translation_D = 0.0;

    public static final double rotation_P = 5.0;
    public static final double rotation_I = 0.0;
    public static final double rotation_D = 0.0;

    // FIXME: All CAN ID's will need to be programmed into the hardware, so we can
    // change these if we want. Personally, I would recommend moving to a more
    // particular numbering scheme. Either way, it would help to have an excel doc
    // or something similar with ID's, PDH port #'s, functions, etc.
    // CAN IDs
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 14;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 13;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 33;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 17;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 18;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 31;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 12;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 34;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 15;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 16;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 32;

    // FIXME: These offsets are from Arno. CRESCENDO will require new offsets be measured
    // Module Offsets - rotational offsets such that the modules all read 0 degrees
    // when facing forward. Added 180 to each at Ruckus because we flipped the arm
    // around so everything is backwards
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.62 + 180.;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -246.09 + 180.;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -288.37 + 180.;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -345.23 + 180.;

    //Limelight/vision
    public static final String limelightName = "limelight";
    public static final double speedLimit = 1.0;

}
