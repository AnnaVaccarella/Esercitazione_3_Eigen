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
  Vector2d solution(-1.0000e+0, -1.0000e+00);  //soluzione esatta dei sistemi lineari da risolvere

  Matrix2d A1 = Matrix2d::Zero();  //inizializzazione della matrice A1 attraverso una matrice nulla di tipo double 2*2
  Vector2d b1 = Vector2d::Zero();  //inizializzazione del termine noto b1 ad un vettore nullo di tipo double formato da due elementi(2*1)

  A1<< 5.547001962252291e-01, -3.770900990025203e-02, 8.320502943378437e-01, -9.992887623566787e-01;
  b1<< -5.169911863249772e-01, 1.672384680188350e-01;
  double errRel1PALU, errRel1QR;
  TestSolution(A1, b1, solution, errRel1PALU, errRel1QR);

  if (errRel1PALU < 1e-15 && errRel1QR < 1e-15)  //controllo che gli errori relativi siano minori di una certa soglia
    cout<< scientific<< "1 - "<< "PALU: "<< errRel1PALU<< " QR: "<< errRel1QR<< endl;
  else
  {
    cerr<< "1 - Wrong system solution found"<< endl;
    return -1;  //Si suppone che tutti e tre i sistemi abbiano soluzione quindi se non riesce a risolvere il
  }             //primo esce dal programma senza provare a risolvere gli altri

 Matrix2d A2 = Matrix2d::Zero();  //inizializzazione di A2 tramite una matrice nulla 2*2
  Vector2d b2 = Vector2d::Zero();  //inizializzazione di b2 tramite un vettore nullo formato da due elementi

  A2<< 5.547001962252291e-01, -5.540607316466765e-01, 8.320502943378437e-01, -8.324762492991313e-01;
  b2<< -6.394645785530173e-04, 4.259549612877223e-04;

  double errRel2PALU, errRel2QR;
  TestSolution(A2, b2, solution, errRel2PALU, errRel2QR);

  if (errRel2PALU < 1e-12 && errRel2QR < 1e-12)  //controllo che l'errore relativo sia minore di una certa soglia
    cout<< scientific<< "2 - "<< "PALU: "<< errRel2PALU<< " QR: "<< errRel2QR<< endl;
  else
  {
    cerr<< "2 - Wrong system solution found"<< endl;
    return -1;
  }