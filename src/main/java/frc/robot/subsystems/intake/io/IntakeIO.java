package frc.robot.subsystems.intake.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.intake.IntakeConstants;
import team6230.koiupstream.io.UpstreamIO;
import team6230.koiupstream.io.UpstreamIO.UpstreamIOInputs;

public abstract class IntakeIO extends UpstreamIO<IntakeIOInputsAutoLogged> {
    protected Rotation2d _targetAngle = IntakeConstants.kMaxAngle;
    protected double _pivotVoltage = 0;

    protected boolean _closedLoopPivot = false;

    @AutoLog
    public static class IntakeIOInputs extends UpstreamIOInputs {
        public double pivotTargetAngle = 0.0;
        public double pivotAngleError = 0.0;
        public double relativePivotAngleRad = 0.0;
        public double absolutePivotAngleRad = 0.0;
        public double relativePivotAngleDeg = 0.0;
        public double absolutePivotAngleDeg = 0.0;
        public double pivotAppliedVoltage = 0.0;
        public double[] pivotCurrent = { 0.0 };
        public boolean pivotClosedLoop = false;
    }

    public IntakeIO() {
        super("IntakePivot");
    }

    public abstract void runVoltsPivot(double volts);

    public abstract void setTargetAngle(Rotation2d angle);

    public abstract Rotation2d getPivotVelocity();

    public abstract Rotation2d getPivotAngle();

    public abstract void stop();

    public abstract void setPIDF(double kP, double kI, double kD, double kS, double kG, double kV, double kA);

    public Rotation2d getTargetAngle() {
        return _targetAngle;
    }
}