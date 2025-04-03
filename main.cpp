#include <iostream>
#include "Eigen/Eigen" //libreria Eigen che serve per operare con matrici e vettori

using namespace std;
using namespace Eigen;

/// \brief Test the real solution of system Ax = b
/// \return the relative error for PALU solver
/// \return the relative error for QR solver
void TestSolution(const MatrixXd& A,
				  const VectorXd& b,
				  const VectorXd& solution,
				  double& errRelPalu,
				  double& errRelQR);

/// \brief Solve linear system with PALU
/// \return the solution
VectorXd SolveSystemPALU(const MatrixXd& A,
                         const VectorXd& b);

/// \brief Solve linear system with PALU
/// \return the solution
VectorXd SolveSystemQR(const MatrixXd& A,
                       const VectorXd& b); 


int main()
{
    return 0;
}
