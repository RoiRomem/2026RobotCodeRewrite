package frc.robot.util.roller;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.roller.io.RollerIO;
import frc.robot.util.roller.io.RollerIOSim;
import frc.robot.util.roller.io.RollerIOSpark;

public class Roller {
    public static RollerIO makeRollerIO(RollerConfig config) {
        if (RobotBase.isReal()) {
            return new RollerIOSpark(config);
        }
        return new RollerIOSim(config);
    }
}
