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
    double rotationSpeed = 0;

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
                List<PhotonTrackedTarget> targets = result.getTargets();
                System.out.println(targets);

                for (int i = 0; i < targets.size(); i++) {
                    int targetID = targets.get(i).getFiducialId();
                    System.out.println("target ID "+targetID);
                    if (targetID == speakerID) {
                        // Calculate angular turn power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                        rotationSpeed = -turnController.calculate(targets.get(i).getYaw(), 0);
                        System.out.println("rotation speed "+rotationSpeed);

                    }
                         
                }
            } 
            else {
                // If we have no targets, stay still.
                rotationSpeed = 0;
            }
            m_swerve.applyRequest(() -> m_drive.withRotationalRate(rotationSpeed));
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
