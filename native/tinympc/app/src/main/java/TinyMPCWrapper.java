public class TinyMPCWrapper {
  static {
    System.loadLibrary("tinympc_jni"); // Load the native library
  }

  // Initialize the TinyMPC solver
  public native void initialize(
      int nStates,
      int nInputs,
      int horizon,
      float rho,
      float[] Adyn,
      float[] Bdyn,
      float[] Q,
      float[] R);

  // Set the initial state for the solver
  public native void setInitialState(float[] initialState);

  // Set the reference trajectory
  public native void setReferenceTrajectory(float[] referenceTrajectory, int horizon);

  // Solve a single MPC step and return the control inputs
  public native float[] solveStep();

  // Clean up solver resources
  public native void cleanup();
}
