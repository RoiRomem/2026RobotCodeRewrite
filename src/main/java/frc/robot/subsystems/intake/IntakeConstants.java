package frc.robot.subsystems.intake;

public class IntakeConstants {
    public final static double kRollerGearRatio = 4;
    public final static double kPivotGearRatio = 25;

    public final static double kMOIpivot = 0.1;
    public final static double kLengthPivot = 0.3;
    public final static double kMinAngle = 6;
    public final static double kMaxAngle = 324;
    public final static double kMinAngleRad = Math.toRadians(kMinAngle);
    public final static double kMaxAngleRad = Math.toRadians(kMaxAngle);

    public final static double kPsim = 0.1, kIsim = 0, kDsim = 0.01;
    public final static double kSsim = 0.144, kVsim = 0.0025, kAsim = 0, kGsim = 0.185;

    public final static double kMaxVelocityRadPerSec = 2 * Math.PI;
    public final static double kMaxAccelRadPerSecSquared = Math.PI;
}
