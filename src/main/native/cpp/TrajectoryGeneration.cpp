#include "TrajectoryGeneration.h"
#include "trajopt/SwerveTrajectoryGenerator.hpp"
#include "trajopt/path/Path.hpp"
#include <vector>

trajopt::SwerveSolution trajgen::CreateInitialGuess(std::vector<trajopt::Waypoint> trajectory) {
  return trajopt::SwerveSolution{};
}

trajopt::SwerveTrajectory trajgen::Generate(InputTrajectory trajectory) {
  return trajopt::SwerveTrajectory{};
}
