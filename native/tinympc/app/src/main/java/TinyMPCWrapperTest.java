public class TinyMPCWrapperTest {
  public static void main(String[] args) {
    TinyMPCWrapper mpc = new TinyMPCWrapper();

    // Example data (replace with actual values as needed)
    int nStates = 4;
    int nInputs = 1;
    int horizon = 10;
    float rho = 1.0f;

    float[] Adyn = {
      1.0f, 0.01f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.039f, 0.0f,
      0.0f, 0.0f, 1.002f, 0.01f,
      0.0f, 0.0f, 0.458f, 1.002f
    };
    float[] Bdyn = {0.0f, 0.02f, 0.0f, 0.067f};
    float[] Q = {10.0f, 1.0f, 10.0f, 1.0f};
    float[] R = {1.0f};
    float[] initialState = {0.5f, 0.0f, 0.0f, 0.0f};
    float[] referenceTrajectory = new float[horizon * nStates];

    // Set a simple reference trajectory
    for (int i = 0; i < horizon; i++) {
      referenceTrajectory[i * nStates] = 1.0f; // x = 1.0
      referenceTrajectory[i * nStates + 1] = 0.0f; // y = 0.0
      referenceTrajectory[i * nStates + 2] = 0.0f; // z = 0.0
      referenceTrajectory[i * nStates + 3] = 0.0f; // yaw = 0.0
    }

    // Initialize the MPC solver
    mpc.initialize(nStates, nInputs, horizon, rho, Adyn, Bdyn, Q, R);

    // Set the initial state and reference trajectory
    mpc.setInitialState(initialState);
    mpc.setReferenceTrajectory(referenceTrajectory, horizon);

    // Solve MPC steps
    for (int step = 0; step < 10; step++) {
      float[] controlInputs = mpc.solveStep();
      System.out.println("Control Inputs: " + java.util.Arrays.toString(controlInputs));
    }

    // Cleanup
    mpc.cleanup();
  }
}
