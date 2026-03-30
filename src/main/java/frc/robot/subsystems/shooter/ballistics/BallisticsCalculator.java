package frc.robot.subsystems.shooter.ballistics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceFlipUtil;

public class BallisticsCalculator {
    public double getScoringSpeed() {
        return BallisticsParameters.kShotFlywheelSpeedMap.get(getShooterDistanceToHub());
    }

    public double getScoringAngle() {
        return BallisticsParameters.kShotHoodAngleMap.get(getShooterDistanceToHub());
    }

    private double getShooterDistanceToHub() {
        var shooterTranslation = getShooterPose().getTranslation();
        var hubTranslation = getAllianceHubCenterTranslation();

        return shooterTranslation.getDistance(hubTranslation);
    }

    private Pose2d getShooterPose() {
        var robotPose = RobotContainer.getRobotPose();
        return robotPose.transformBy(new Transform2d(
                BallisticsParameters.kShooterOffset.getX(),
                BallisticsParameters.kShooterOffset.getY(),
                new Rotation2d()));
    }

    private Translation2d getAllianceHubCenterTranslation() {
        return AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint).toTranslation2d();
    }
}
