package frc.robot.subsystems.intake.io;

import org.littletonrobotics.junction.AutoLog;

import team6230.koiupstream.io.UpstreamIO;
import team6230.koiupstream.io.UpstreamIO.UpstreamIOInputs;

public abstract class IntakeIO extends UpstreamIO<IntakeIOInputsAutoLogged> {
    protected double _targetAngle = Double.NaN;
    protected double _targetRPM = Double.NaN;

    protected boolean _closedLoopPivot = false;
    protected boolean _closedLoopRoller = false;

    @AutoLog
    public static class IntakeIOInputs extends UpstreamIOInputs {
        public double relativePivotAngle = 0.0;
        public double absolutePivotAngle = 0.0;
        public double pivotAppliedVoltage = 0.0;
        public double[] pivotCurrent = { 0.0 };
        public boolean pivotClosedLoop = false;
        public double rollerRPM = 0.0;
        public double rollerAppliedVoltage = 0.0;
        public double[] rollerCurrent = { 0.0 };
        public boolean rollerClosedLoop = false;
    }

    public IntakeIO() {
        super("Intake");
    }

    public abstract void runVoltsArm(double volts);

    public abstract void runVoltsRollers(double volts);

    public abstract void setTargetAngle(double angle);

    public double getTargetAngle() {
        return _targetAngle;
    }

    public abstract void setTargetRPM(double rpm);

    public double getTargetRPM() {
        return _targetRPM;
    }
}
