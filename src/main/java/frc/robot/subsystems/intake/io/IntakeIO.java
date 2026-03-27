package frc.robot.subsystems.intake.io;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.IntakeConstants;
import team6230.koiupstream.io.UpstreamIO;
import team6230.koiupstream.io.UpstreamIO.UpstreamIOInputs;

public abstract class IntakeIO extends UpstreamIO<IntakeIOInputsAutoLogged> {
    protected double _targetAngle = IntakeConstants.kMinAngle;
    protected double _pivotVoltage = 0;
    protected double _rollerVoltage = 0;

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
        public double rollerRPM = 0.0;
        public double rollerAppliedVoltage = 0.0;
        public double[] rollerCurrent = { 0.0 };
    }

    public IntakeIO() {
        super("Intake");
    }

    public abstract void runVoltsPivot(double volts);

    public abstract void runVoltsRollers(double volts);

    public abstract void setTargetAngle(double angle);

    public abstract double getPivotVelocity();

    public abstract double getPivotAngleDeg();

    public abstract void stop();

    public double getTargetAngle() {
        return _targetAngle;
    }
}
