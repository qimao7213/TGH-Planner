#pragma once

#include <osqp/osqp.h>

#include <Eigen/Sparse>
#include <memory>

/**
 * osqp interface:
 * minimize     0.5 x' P x + q' x
 * subject to   l <= A x <= u
 **/

namespace osqp {

class IOSQP {
 public:
  IOSQP() : UNBOUNDED_VAL(OSQP_INFTY),
            pWork(nullptr),
            pSettings(nullptr),
            pData(nullptr) {
    pSettings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    pData = (OSQPData *)c_malloc(sizeof(OSQPData));
    if (pSettings)
      osqp_set_default_settings(pSettings);
  }

  ~IOSQP() {
    if (pWork)
      osqp_cleanup(pWork);
    if (pSettings)
      c_free(pSettings);
    if (pData)
      c_free(pData);
  }

  const double UNBOUNDED_VAL;

  inline c_int setMats(Eigen::SparseMatrix<double> &P,
                       Eigen::VectorXd &q,
                       Eigen::SparseMatrix<double> &A,
                       Eigen::VectorXd &l,
                       Eigen::VectorXd &u,
                       const double &eps_abs = 1e-3,
                       const double &eps_rel = 1e-3) {
    if (pWork)
      osqp_cleanup(pWork);

    P = P.triangularView<Eigen::Upper>();
    P.makeCompressed();
    A.makeCompressed();

    pData->n = P.rows();
    pData->m = A.rows();

    Eigen::Map<const Eigen::VectorXi> iIdxP(P.innerIndexPtr(), P.nonZeros());
    Eigen::Map<const Eigen::VectorXi> oIdxP(P.outerIndexPtr(), P.cols() + 1);
    Eigen::Matrix<c_int, -1, 1> innerIdxP = iIdxP.cast<c_int>();
    Eigen::Matrix<c_int, -1, 1> outerIdxP = oIdxP.cast<c_int>();
    pData->P = csc_matrix(pData->n,
                          pData->n,
                          P.nonZeros(),
                          P.valuePtr(),
                          innerIdxP.data(),
                          outerIdxP.data());

    Eigen::Map<const Eigen::VectorXi> iIdxA(A.innerIndexPtr(), A.nonZeros());
    Eigen::Map<const Eigen::VectorXi> oIdxA(A.outerIndexPtr(), A.cols() + 1);
    Eigen::Matrix<c_int, -1, 1> innerIdxA = iIdxA.cast<c_int>();
    Eigen::Matrix<c_int, -1, 1> outerIdxA = oIdxA.cast<c_int>();
    pData->A = csc_matrix(pData->m,
                          pData->n,
                          A.nonZeros(),
                          A.valuePtr(),
                          innerIdxA.data(),
                          outerIdxA.data());

    Eigen::Matrix<c_float, -1, 1> pDqVec = q.cast<c_float>();
    pData->q = pDqVec.data();
    Eigen::Matrix<c_float, -1, 1> pDlVec = l.cast<c_float>();
    pData->l = pDlVec.data();
    Eigen::Matrix<c_float, -1, 1> pDuVec = u.cast<c_float>();
    pData->u = pDuVec.data();

    pSettings->eps_abs = eps_abs;
    pSettings->eps_rel = eps_rel;

    c_int exitflag = osqp_setup(&pWork, pData, pSettings);

    if (pData->A)
      c_free(pData->A);
    if (pData->P)
      c_free(pData->P);

    return exitflag;
  }

  inline c_int solve() const {
    return osqp_solve(pWork);
  }

  inline c_int getStatus() const {
    return pWork->info->status_val;
  }

  inline Eigen::VectorXd getPrimalSol() const {
    return Eigen::Map<const Eigen::VectorXd>(pWork->solution->x,
                                             pWork->data->n);
  }

 private:
  OSQPWorkspace *pWork;
  OSQPSettings *pSettings;
  OSQPData *pData;
};

}  // namespace osqp