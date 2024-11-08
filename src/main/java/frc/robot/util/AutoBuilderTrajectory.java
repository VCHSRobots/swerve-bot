package frc.robot.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoBuilderTrajectory extends AutoBuilder {

  public static class AutoOption {
    public PathPlannerAuto command;
    public Trajectory trajectory;

    public AutoOption(PathPlannerAuto command, Trajectory trajectory) {
      this.command = command;
      this.trajectory = trajectory;
    }
  }

  /**
   * Create and populate a sendable chooser with all PathPlannerAutos in the
   * project
   *
   * @param defaultAutoName The name of the auto that should be the default
   *                        option. If this is an
   *                        empty string, or if an auto with the given name does
   *                        not exist, the default option will be
   *                        Commands.none()
   * @return SendableChooser populated with all autos
   */
  public static SendableChooser<AutoOption> buildAutoChooserWithTrajectories(String defaultAutoName) {
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build an auto chooser");
    }

    SendableChooser<AutoOption> chooser = new SendableChooser<>();
    List<String> autoNames = getAllAutoNames();

    AutoOption defaultOption = null;
    List<AutoOption> options = new ArrayList<>();

    for (String autoName : autoNames) {
      AutoOption auto = new AutoOption(new PathPlannerAuto(autoName),
          trajectoryFromAuto(autoName));

      if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
        defaultOption = auto;
      } else {
        options.add(auto);
      }
    }

    if (defaultOption == null) {
      chooser.setDefaultOption("None", new AutoOption(null, new Trajectory()));
    } else {
      chooser.setDefaultOption(defaultOption.command.getName(), defaultOption);
    }

    options.forEach(auto -> chooser.addOption(auto.command.getName(), auto));

    return chooser;
  }

  public static Trajectory trajectoryFromAuto(String autoName) {
    List<Pose2d> pathPoses = new ArrayList<>();
    for (PathPlannerPath path : pathPlannerPathsFromAuto(autoName)) {
      pathPoses.addAll(path.getPathPoses());
    }
    return TrajectoryGenerator.generateTrajectory(pathPoses, new TrajectoryConfig(3, 3));
  }

  public static List<PathPlannerPath> pathPlannerPathsFromAuto(String autoName) {
    ArrayList<PathPlannerPath> paths = new ArrayList<>();
    try (BufferedReader br = new BufferedReader(
        new FileReader(
            new File(
                Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto")))) {
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();
      JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

      boolean choreoAuto = json.get("choreoAuto") != null && (boolean) json.get("choreoAuto");
      ArrayList<JSONObject> cmdStack = new ArrayList<>();
      cmdStack.add((JSONObject) json.get("command"));

      while (!cmdStack.isEmpty()) {
        JSONObject currCmd = cmdStack.remove(0);
        JSONObject data = (JSONObject) currCmd.get("data");
        String type = (String) currCmd.get("type");

        switch (type) {
            case "path":
            String pathName = (String) data.get("pathName");
            paths.add(choreoAuto
                ? PathPlannerPath.fromChoreoTrajectory(pathName)
                : PathPlannerPath.fromPathFile(pathName));
          case "wait":
          case "named":
            continue;
          case "sequential":
          case "parallel":
          case "race":
          case "deadline":
            for (var cmdJson : (JSONArray) data.get("commands")) {
              cmdStack.add((JSONObject) cmdJson);
            }
        }
      }
    } catch (Exception e) {
      throw new RuntimeException(String.format("Error building auto: %s", autoName), e);
    }
    return paths;
  }
}