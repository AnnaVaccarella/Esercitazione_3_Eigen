#include <iostream>
#include "Eigen/Eigen"


using namespace std;
using namespace Eigen;

/// \brief Test the real solution of system Ax = b
/// \return the relative error for PALU solver
/// \return the relative error for QR solver
void TestSolution(const MatrixXd& A,
                  const VectorXd& b,
                  const VectorXd& solution,
                  double& errRelPALU,
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

  Matrix2d A3 = Matrix2d::Zero();  //inizializzazione di A3 tramite una matrice nulla 2*2
  Vector2d b3 = Vector2d::Zero();  //inizializzazione di b3 tramite un vettore nullo formato da due elementi

  A3<< 5.547001962252291e-01, -5.547001955851905e-01, 8.320502943378437e-01, -8.320502947645361e-01;
  b3<< -6.400391328043042e-10, 4.266924591433963e-10;

  double errRel3PALU, errRel3QR;

  TestSolution(A3, b3, solution, errRel3PALU, errRel3QR);

  if (errRel3PALU < 1e-5 && errRel3QR < 1e-5)  //controllo che l'errore relativo sia minore di una certa soglia
    cout<< scientific<< "3 - "<< "PALU: "<< errRel3PALU<< " QR: "<< errRel3QR<< endl;
  else
  {
    cerr<< "3 - Wrong system solution found"<< endl;
    return -1;
  }

  return 0;
}
void TestSolution(const MatrixXd& A,
                  const VectorXd& b,
                  const VectorXd& solution,
                  double& errRelPALU,
                  double& errRelQR)
{
    double detA = A.determinant();
    //errRelPALU=-1;
    //errRelQR =-1;
    errRelPALU = numeric_limits<double>::max();  //serve come inizializzazione dell'errore al posto di usare -1, è un valore molto alto, improbabile
    errRelQR = numeric_limits<double>::max();    //così se la matrice è singolare non può calcolare l'errore ma comunque non resta senza inizializzazione
    if( abs(detA)< 1e-16)  //controllo se la matrice è singolare verificando se il determinante è minore di una certa soglia
    {                      //in alternativa si poteva verificare se il minimo dei valori singolari era prossimo a 0
      cout<<"The matrix is singular"<<endl;
    }
    else
    {
        Vector2d x=Vector2d::Zero();  //inizializzazione del vettore ottenuto come soluzione del sistema lineare
        x=SolveSystemPALU(A,b);  //invoco la funzione che risolve il sistema lineare usando la fattorizzazione PA=LU
        errRelPALU= (solution - x).norm() / solution.norm();  //calcolo in norma euclidea l'errore relativo tra la soluzione
                                                             //ottenuta dalla fattorizzazione PA=LU e la soluzione esatta
        x=SolveSystemQR(A,b);  //invoco la funzione che risolve il sistema lineare usando la fattorizzazione QR
        errRelQR= (solution - x).norm() / solution.norm();  //calcolo in norma euclidea l'errore relativo tra la soluzione
                                                           //ottenuta dalla fattorizzazione QR e la soluzione esatta
    }
}