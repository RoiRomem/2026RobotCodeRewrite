package frc.robot.subsystems.shooter.flywheelIO;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim flywheelSim;

    public ShooterIOSim() {
        DCMotor gearbox = DCMotor.getNeoVortex(2);

        LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
                gearbox,
                0.004,
                1.0);

        flywheelSim = new FlywheelSim(plant, gearbox, 0.0001);
    }
}
