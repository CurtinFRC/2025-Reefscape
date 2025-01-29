#include <jni.h>
#include "tiny_api.hpp"
#include <iostream>
#include <vector>
#include <sstream>
#include <stdexcept>
#include "fmt/base.h"

// Global solver instance
static TinySolver* solver = nullptr;

// RAII class for managing JNI arrays safely
class JArrayGuard {
    JNIEnv* env;
    jfloatArray array;
    jfloat* elements;

public:
    JArrayGuard(JNIEnv* env, jfloatArray array)
        : env(env), array(array), elements(env->GetFloatArrayElements(array, nullptr)) {
        if (!elements) {
            throw std::runtime_error("Failed to get elements from JNI array.");
        }
    }

    ~JArrayGuard() { env->ReleaseFloatArrayElements(array, elements, JNI_ABORT); }

    jfloat* get() const { return elements; }
};

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_org_curtinfrc_frc2025_TinyMPCJNI_test(JNIEnv* env) {
  fmt::println("Hello world!");
}

JNIEXPORT void JNICALL Java_org_curtinfrc_frc2025_TinyMPCJNI_initialize(
    JNIEnv* env, jobject obj,
    jint nStates, jint nInputs, jint horizon, jfloat rho,
    jfloatArray AdynArr, jfloatArray BdynArr,
    jfloatArray QArr, jfloatArray RArr) {
    
    if (solver != nullptr) {
        delete solver;
        solver = nullptr;
    }

    // Validate array sizes
    if (env->GetArrayLength(AdynArr) != nStates * nStates ||
        env->GetArrayLength(BdynArr) != nStates * nInputs ||
        env->GetArrayLength(QArr) != nStates ||
        env->GetArrayLength(RArr) != nInputs) {
        throw std::runtime_error("Array dimensions do not match expected sizes.");
    }

    // Use RAII for safe JNI resource management
    JArrayGuard adynGuard(env, AdynArr);
    JArrayGuard bdynGuard(env, BdynArr);
    JArrayGuard qGuard(env, QArr);
    JArrayGuard rGuard(env, RArr);

    // Map JNI arrays to Eigen matrices and cast to double
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> AdynFloat(adynGuard.get(), nStates, nStates);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> BdynFloat(bdynGuard.get(), nStates, nInputs);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>> QVecFloat(qGuard.get(), nStates);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>> RVecFloat(rGuard.get(), nInputs);

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
        delete solver;
        solver = nullptr;
        std::ostringstream errorMsg;
        errorMsg << "Failed to initialize TinyMPC solver. Error code: " << status;
        throw std::runtime_error(errorMsg.str());
    }
}

JNIEXPORT void JNICALL Java_org_curtinfrc_frc2025_TinyMPCJNI_setInitialState(JNIEnv* env, jobject obj, jfloatArray initialStateArr) {
    if (solver == nullptr) {
        throw std::runtime_error("Solver is not initialized. Call initialize() first.");
    }

    JArrayGuard initialStateGuard(env, initialStateArr);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>> x0Float(initialStateGuard.get(), solver->work->nx);
    Eigen::VectorXd x0 = x0Float.cast<double>();

    tiny_set_x0(solver, x0);
}

JNIEXPORT void JNICALL Java_org_curtinfrc_frc2025_TinyMPCJNI_setReferenceTrajectory(JNIEnv* env, jobject obj, jfloatArray referenceTrajectoryArr, jint horizon) {
    if (solver == nullptr) {
        throw std::runtime_error("Solver is not initialized. Call initialize() first.");
    }

    JArrayGuard referenceTrajectoryGuard(env, referenceTrajectoryArr);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> XrefFloat(
        referenceTrajectoryGuard.get(), solver->work->nx, horizon);
    Eigen::MatrixXd Xref = XrefFloat.cast<double>();

    solver->work->Xref = Xref;
}

JNIEXPORT jfloatArray JNICALL Java_org_curtinfrc_frc2025_TinyMPCJNI_solveStep(JNIEnv* env, jobject obj) {
    if (solver == nullptr) {
        throw std::runtime_error("Solver is not initialized. Call initialize() first.");
    }

    tiny_solve(solver);

    const int nInputs = solver->work->nu;
    std::vector<float> controlInputs(nInputs);
    for (int i = 0; i < nInputs; ++i) {
        controlInputs[i] = static_cast<float>(solver->work->u.col(0)(i));
    }

    jfloatArray result = env->NewFloatArray(nInputs);
    if (result == nullptr) {
        throw std::runtime_error("Failed to allocate result array.");
    }
    env->SetFloatArrayRegion(result, 0, nInputs, controlInputs.data());
    return result;
}

JNIEXPORT void JNICALL Java_org_curtinfrc_frc2025_TinyMPCJNI_cleanup(JNIEnv* env, jobject obj) {
    if (solver != nullptr) {
        delete solver;
        solver = nullptr;
    }
}

#ifdef __cplusplus
} // extern "C"
#endif