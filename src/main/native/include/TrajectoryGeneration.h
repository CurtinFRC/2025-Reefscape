#pragma once

#include "trajopt/SwerveTrajectoryGenerator.hpp"
#include "trajopt/constraint/Constraint.hpp"
#include "trajopt/path/Path.hpp"
#include <vector>

namespace trajgen {
struct InputTrajectory {
  std::vector<trajopt::Waypoint> waypoints;
  std::vector<trajopt::Constraint> constraints;
};

trajopt::SwerveSolution CreateInitialGuess(std::vector<trajopt::Waypoint> waypoints);
trajopt::SwerveTrajectory Generate(InputTrajectory trajectory);
}
