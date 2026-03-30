package frc.robot.subsystems.shooter.hoodIO;

import frc.robot.Constants;

public class Hood {
    public static HoodIO getHoodIO() {
        switch (Constants.currentMode) {
            case REAL:
                return new HoodIOReal();

            case SIM:
                return new HoodIOSim();

            case REPLAY:
                return new HoodIO() {
                };
        }
        return new HoodIO() {
        };
    }
}
