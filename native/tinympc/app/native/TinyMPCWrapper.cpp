#include <jni.h>
#include "tinympc/tiny_api.hpp"
#include <iostream>
#include <vector>

// Global solver instance
static TinySolver* solver = nullptr;

extern "C" {

JNIEXPORT void JNICALL Java_TinyMPCWrapper_initialize(
    JNIEnv* env, jobject obj,
    jint nStates, jint nInputs, jint horizon, jfloat rho,
    jfloatArray AdynArr, jfloatArray BdynArr,
    jfloatArray QArr, jfloatArray RArr) {
    
    // Convert Java arrays to native arrays
    jfloat* Adyn = env->GetFloatArrayElements(AdynArr, nullptr);
    jfloat* Bdyn = env->GetFloatArrayElements(BdynArr, nullptr);
    jfloat* Q = env->GetFloatArrayElements(QArr, nullptr);
    jfloat* R = env->GetFloatArrayElements(RArr, nullptr);

    // Map to Eigen matrices and cast to double
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> AdynFloat(Adyn, nStates, nStates);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> BdynFloat(Bdyn, nStates, nInputs);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>> QVecFloat(Q, nStates);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>> RVecFloat(R, nInputs);

    Eigen::MatrixXd AdynMat = AdynFloat.cast<double>();
    Eigen::MatrixXd BdynMat = BdynFloat.cast<double>();
    Eigen::VectorXd QVec = QVecFloat.cast<double>();
    Eigen::VectorXd RVec = RVecFloat.cast<double>();

    // Initialize constraints
    Eigen::MatrixXd xMin = Eigen::MatrixXd::Constant(nStates, horizon, -1e17);
    Eigen::MatrixXd xMax = Eigen::MatrixXd::Constant(nStates, horizon, 1e17);
    Eigen::MatrixXd uMin = Eigen::MatrixXd::Constant(nInputs, horizon - 1, -1e17);
    Eigen::MatrixXd uMax = Eigen::MatrixXd::Constant(nInputs, horizon - 1, 1e17);

    // Setup solver
    solver = new TinySolver();
    int status = tiny_setup(&solver, AdynMat, BdynMat, QVec.asDiagonal(), RVec.asDiagonal(),
                            rho, nStates, nInputs, horizon, xMin, xMax, uMin, uMax, 1);

    if (status != 0) {
        throw std::runtime_error("Failed to initialize TinyMPC solver.");
    }

    // Release JNI arrays
    env->ReleaseFloatArrayElements(AdynArr, Adyn, JNI_ABORT);
    env->ReleaseFloatArrayElements(BdynArr, Bdyn, JNI_ABORT);
    env->ReleaseFloatArrayElements(QArr, Q, JNI_ABORT);
    env->ReleaseFloatArrayElements(RArr, R, JNI_ABORT);
}

JNIEXPORT void JNICALL Java_TinyMPCWrapper_setInitialState(JNIEnv* env, jobject obj, jfloatArray initialStateArr) {
    jfloat* initialState = env->GetFloatArrayElements(initialStateArr, nullptr);

    // Convert JNI array to Eigen vector and cast to double
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>> x0Float(initialState, solver->work->nx);
    Eigen::VectorXd x0 = x0Float.cast<double>();

    tiny_set_x0(solver, x0);

    // Release JNI array
    env->ReleaseFloatArrayElements(initialStateArr, initialState, JNI_ABORT);
}

JNIEXPORT void JNICALL Java_TinyMPCWrapper_setReferenceTrajectory(JNIEnv* env, jobject obj, jfloatArray referenceTrajectoryArr, jint horizon) {
    jfloat* referenceTrajectory = env->GetFloatArrayElements(referenceTrajectoryArr, nullptr);

    // Convert JNI array to Eigen matrix and cast to double
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> XrefFloat(
        referenceTrajectory, solver->work->nx, horizon);
    Eigen::MatrixXd Xref = XrefFloat.cast<double>();

    solver->work->Xref = Xref;

    // Release JNI array
    env->ReleaseFloatArrayElements(referenceTrajectoryArr, referenceTrajectory, JNI_ABORT);
}

JNIEXPORT jfloatArray JNICALL Java_TinyMPCWrapper_solveStep(JNIEnv* env, jobject obj) {
    tiny_solve(solver);

    // Get control inputs
    const int nInputs = solver->work->nu;
    std::vector<float> controlInputs(nInputs);
    for (int i = 0; i < nInputs; ++i) {
        controlInputs[i] = static_cast<float>(solver->work->u.col(0)(i));
    }

    // Create JNI array and copy results
    jfloatArray result = env->NewFloatArray(nInputs);
    env->SetFloatArrayRegion(result, 0, nInputs, controlInputs.data());
    return result;
}

JNIEXPORT void JNICALL Java_TinyMPCWrapper_cleanup(JNIEnv* env, jobject obj) {
    delete solver;
    solver = nullptr;
}

} // extern "C"
