// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.util;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.Trajectory;
import org.wpilib.math.trajectory.Trajectory.State;
import java.util.List;
import java.util.Optional;

public class TrajectoryConverter {

  public static Optional<Trajectory> fromPathplanner(
      String name, ChassisVelocities startSpeeds, Rotation2d startRotation, RobotConfig config) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      PathPlannerTrajectory traj =
          new PathPlannerTrajectory(path, startSpeeds, startRotation, config);
      List<State> states =
          traj.getStates().stream()
              .map((x -> new State(x.timeSeconds, x.linearVelocity, 0.0, x.pose, 0)))
              .toList();
      return Optional.of(new Trajectory(states));
    } catch (Exception e) {
      return Optional.empty();
    }
  }
}
