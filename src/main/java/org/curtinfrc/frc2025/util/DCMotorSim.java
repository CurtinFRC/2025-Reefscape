package org.curtinfrc.frc2025.util;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class DCMotorSim extends LinearSystemSim<N2, N1, N2> {
  public static LinearSystem<N2, N1, N2> createDCMotor(double moi, double kt) {
    return new LinearSystem<N2, N1, N2>(
        MatBuilder.fill(Nat.N2(), Nat.N2(), 0, 1, 0, 0),
        VecBuilder.fill(0, kt / moi),
        Matrix.eye(Nat.N2()),
        new Matrix<>(Nat.N2(), Nat.N1()));
  }

  public DCMotorSim(LinearSystem<N2, N1, N2> plant, double... measurementStdDevs) {
    super(plant, measurementStdDevs);
  }
}
