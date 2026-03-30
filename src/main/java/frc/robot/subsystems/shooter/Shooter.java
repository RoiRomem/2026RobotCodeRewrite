package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.io.ShooterIO;
import frc.robot.subsystems.shooter.io.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.shooter.io.ShooterIORev;
import frc.robot.subsystems.shooter.io.ShooterIOSim;
import team6230.koiupstream.subsystems.UpstreamSubsystem;

public class Shooter extends UpstreamSubsystem<RobotState, ShooterIO, ShooterIOInputsAutoLogged> {

    public Shooter() {
        super("Shooter", new ShooterIOInputsAutoLogged());
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isReady() {
        return true;
    }

    @Override
    protected ShooterIO getIO() {
        switch (Constants.currentMode) {
            case REAL:
                return new ShooterIORev();
            case SIM:
                return new ShooterIOSim();
            case REPLAY:
                return new ShooterIO() {
                };
        }
        return new ShooterIO() {
        };
    }

}
