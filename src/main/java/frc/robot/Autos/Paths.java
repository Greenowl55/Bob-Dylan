package frc.robot.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

public class Paths {
    public static final PathPlannerPath move = PathPlannerPath.fromPathFile("Move");
    public static final PathPlannerPath spillyspin = PathPlannerPath.fromPathFile("Funny spinny");
    public static final PathPlannerPath moveandrotate = PathPlannerPath.fromPathFile("Move and Rotate");
    public static final PathPlannerPath oneMeter = PathPlannerPath.fromChoreoTrajectory("oneMeter.1");
    public static final PathPlannerPath trap = PathPlannerPath.fromChoreoTrajectory("ToTrap");
}
