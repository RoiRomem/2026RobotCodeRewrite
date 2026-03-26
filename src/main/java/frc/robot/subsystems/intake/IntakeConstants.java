package frc.robot.subsystems.intake;

public class IntakeConstants {
    public final static double kRollerGearRatio = 4;
    public final static double kPivotGearRatio = 25;

    public final static double kErrorToleranceDeg = 2;

    public static final int kOpenAngle = 323;
    public static final int kMinOpenAngle = 317;
    public static final int kClosedAngle = 7;

    public static final double kEncoderOffsetSim = 5.0;
    public static final double kEncoderToArmRatioSim = (325.0 - 5.0) / 90.0;

    public static final int kIntakingVolts = 12;

    public final static double kMOIpivot = 0.1;
    public final static double kLengthPivot = 0.3;
    public final static double kMinAngle = 6;
    public final static double kMaxAngle = 324;
    public final static double kMinAngleRad = Math.toRadians(kMinAngle);
    public final static double kMaxAngleRad = Math.toRadians(kMaxAngle);

    public final static double kPsim = 1, kIsim = 0, kDsim = 0.1;
    public final static double kSsim = 1.44, kVsim = 0.025, kAsim = 0, kGsim = 3;

    public final static double kMaxVelocityRadPerSec = 2 * Math.PI;
    public final static double kMaxAccelRadPerSecSquared = Math.PI;
}
