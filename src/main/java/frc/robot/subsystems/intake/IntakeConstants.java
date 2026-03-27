package frc.robot.subsystems.intake;

public class IntakeConstants {
    public final static double kRollerGearRatio = 4;
    public final static double kPivotGearRatio = 25;

    public final static double kErrorToleranceDeg = 2;

    public static final double kMinOpenAngle = 317;
    public static final double kEncoderOpen = 323;
    public static final double kEncoderClosed = 7;

    public static final double kPivotMinDeg = 0.0;
    public static final double kPivotMaxDeg = 90.0;

    public static final double kEncoderOffsetSim = 90;
    public static final double kEncoderToPivotRatio = (kEncoderOpen - kEncoderClosed) / (kPivotMaxDeg - kPivotMinDeg);

    public static final double kThroughBoreRange = 360;
    public static final double kThroughBoreOffset = 160;

    public static final double kForwardSoftLimit = 324;
    public static final double kReverseSoftLimit = 6;

    public static double encoderToPivot(double encoderDeg) {
        return (encoderDeg - kEncoderClosed) / kEncoderToPivotRatio;
    }

    public static double pivotToEncoder(double armDeg) {
        return (armDeg * kEncoderToPivotRatio) + kEncoderClosed;
    }

    public static final int kIntakingVolts = 12;

    public final static double kMOIpivot = 0.1;
    public final static double kLengthPivot = 0.3;
    public final static double kMinAngle = 6;
    public final static double kMaxAngle = 324;
    public final static double kMinAngleRad = Math.toRadians(kMinAngle);
    public final static double kMaxAngleRad = Math.toRadians(kMaxAngle);

    public final static double kGroundParallelRad = 5.67232;

    public static final double kMinPhysicalAngleDeg = 5 / kEncoderToPivotRatio;
    public static final double kMaxPhysicalAngleDeg = 325 / kEncoderToPivotRatio;
    public static final double kMinPhysicalAngleRad = Math.toRadians(kMinPhysicalAngleDeg);
    public static final double kMaxPhysicalAngleRad = Math.toRadians(kMaxPhysicalAngleDeg);

    public static final double kP = 0.0001, kI = 0, kD = 0;
    public static final double kS = 0.144, kV = 0.0025, kA = 0, kG = 0.185;
    public static final double kCosRatio = 1;
    public static final double kMaxAcceleration = 46000;
    public static final double kCruiseVelocity = 400000;
    public static final double kTolerance = 0.5;

    public static final int kPivotCurrentLimits = 20;
    public static final int kRollerCurrentLimits = 40;

    public final static double kPsim = 0.01;
    public final static double kIsim = 0;
    public final static double kDsim = 0;
    public final static double kSsim = 0.05;
    public final static double kGsim = 1.5;
    public final static double kVsim = 0;
    public final static double kAsim = 0;

    public final static double kMaxVelocityRadPerSec = 8 * Math.PI;
    public final static double kMaxAccelRadPerSecSquared = 24 * Math.PI;
}
