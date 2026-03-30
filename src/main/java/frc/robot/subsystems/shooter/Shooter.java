package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.flywheelIO.ShooterIO;
import frc.robot.subsystems.shooter.flywheelIO.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheelIO.ShooterIORev;
import frc.robot.subsystems.shooter.flywheelIO.ShooterIOSim;
import frc.robot.subsystems.shooter.hoodIO.Hood;
import frc.robot.subsystems.shooter.hoodIO.HoodIO;
import frc.robot.subsystems.shooter.hoodIO.HoodIOInputsAutoLogged;
import frc.robot.util.roller.Roller;
import frc.robot.util.roller.RollerConfig;
import frc.robot.util.roller.io.RollerIO;
import frc.robot.util.roller.io.RollerIOInputsAutoLogged;
import team6230.koiupstream.subsystems.ExtraIO;
import team6230.koiupstream.subsystems.UpstreamSubsystem;

public class Shooter extends UpstreamSubsystem<RobotState, ShooterIO, ShooterIOInputsAutoLogged> {
    private HoodIO hoodIO = Hood.getHoodIO();
    private HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

    private RollerIO rollerIO;
    private RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

    public Shooter() {
        super("Shooter", new ShooterIOInputsAutoLogged());

        RollerConfig r_config = new RollerConfig();
        r_config.motorId = RobotMap.CanBus.kFeederRollerID;
        r_config.gearRatio = 4;
        r_config.name = "ShooterFeeder";
        r_config.smartCurrentLimit = 60;
        r_config.motor = RollerConfig.RollerMotor.NEO;

        rollerIO = Roller.makeRollerIO(r_config);

        addAnExtraIO(new ExtraIO(hoodIO, hoodInputs, "ShooterHood"));
        addAnExtraIO(new ExtraIO(rollerIO, rollerInputs, r_config.name));
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
