package frc.robot.commands;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.vision.Fiducials;


public class AutoAimPID extends Command{
    private final CommandSwerveDrivetrain m_swerve;
    private final SwerveRequest.FieldCentric m_drive;
    private final PhotonCamera m_camera;
    private final PIDController turnController = new PIDController(1.0, 0, 0.1);
    
    final double MaxAngularRate = 2 * Math.PI;

    public AutoAimPID(CommandSwerveDrivetrain swerve, PhotonCamera camera, SwerveRequest.FieldCentric drive) {
        m_swerve = swerve;
        m_camera = camera;
        m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);

    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
    
  }

  @Override
  public void execute() {
    // Called every time Command is scheduled
    var result = m_camera.getLatestResult();
    var alliance = DriverStation.getAlliance();
    
    int speakerID = 7;

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            speakerID = Fiducials.AprilTags.aprilTagFiducials[6].getID();
        }

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            speakerID = Fiducials.AprilTags.aprilTagFiducials[3].getID();
        }
    
    if (result.hasTargets()) {
      var targetList = result.getTargets();
      for (int i = 0; i < targetList.size(); i++) {
        var target = targetList.get(i);

        if (target.getFiducialId() == speakerID) {
          double rotationSpeed = -turnController.calculate(target.getYaw(), 0);
          m_swerve.applyRequest(() -> m_drive.withRotationalRate(rotationSpeed*MaxAngularRate));

        }
      }
    }    
  }

  @Override
  public void end(boolean interrupted) {
    //Called when command ends or is interrupted
  }

  @Override
  public boolean isFinished() {
    //Called when Command is finished
    return false;
  }
    


    
}
