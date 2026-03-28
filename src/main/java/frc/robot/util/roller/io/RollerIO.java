package frc.robot.util.roller.io;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.roller.RollerConfig;
import team6230.koiupstream.io.UpstreamIO;
import team6230.koiupstream.io.UpstreamIO.UpstreamIOInputs;

public abstract class RollerIO extends UpstreamIO<RollerIOInputsAutoLogged> {
    @AutoLog
    public static class RollerIOInputs extends UpstreamIOInputs {
        public double appliedVoltage = 0.0;
        public double radsPerSec = 0.0;
        public double[] rollerCurrent = { 0.0 };
    }

    public RollerIO(RollerConfig config) {
        super(config.name);
    }

    public abstract void runVoltage(double volt);

    public abstract double getRollerRadsPerSec();
}
