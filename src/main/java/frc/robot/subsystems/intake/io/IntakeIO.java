package frc.robot.subsystems.intake.io;

import org.littletonrobotics.junction.AutoLog;

import team6230.koiupstream.io.UpstreamIO;
import team6230.koiupstream.io.UpstreamIO.UpstreamIOInputs;

public abstract class IntakeIO extends UpstreamIO<IntakeIOInputsAutoLogged> {

    @AutoLog
    public static class IntakeIOInputs extends UpstreamIOInputs {
        public double relativePivotAngle = 0.0;
        public double absolutePivotAngle = 0.0;
        public double pivotAppliedVoltage = 0.0;
        public double[] pivotCurrent = { 0.0 };
        public double rollerRPM = 0.0;
        public double rollerAppliedVoltage = 0.0;
        public double[] rollerCurrent = { 0.0 };
    }

    public IntakeIO() {
        super("Intake");
    }
}
