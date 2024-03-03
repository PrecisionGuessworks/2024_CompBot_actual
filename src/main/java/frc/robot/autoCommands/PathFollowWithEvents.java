package frc.robot.autoCommands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class PathFollowWithEvents extends Command{
    private Command pathFollowCommand;
    private List<Pair<Double, Command>> unstartedCommands;
    private List<Pair<Double, Command>> pauseTimes;
    private List<Command> runningCommands;
    private Timer timer;
    private boolean paused;
    

    public PathFollowWithEvents(Command pathFollowCommand, PathPlannerTrajectory path) {
        m_requirements.addAll(pathFollowCommand.getRequirements());

        this.unstartedCommands = new ArrayList<>(path.getEventCommands());
        this.unstartedCommands.sort

    }
    
}
