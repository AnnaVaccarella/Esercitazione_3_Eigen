#include <iostream>
#include <Eigen>

using namespace std;
using namespace Eigen;
// PALU
Vector2d solveWithPALU (const Matrix2d& A, const Vector2d& b) {
	PartialPivLU<Matrix2d> lu(A);
	Vector2d x = lu.solve(b);
	cout << "Soluzione con PALU: " << x.transpose() << endl;
	return x;
}

// QR
Vector2d solveWithQR (const Matrix2d& A, const Vector2d& b) {
	HouseHolderQR<Matrix2d> qr(A);
	Vector2d x = qr.solve(b);
	cout << "Soluzione con QR: " << x.transpose() << endl;
	return x;
}

// calcola errore relativo
void calculateError(const Vector2d& x, const Vector2d& expected) {
	double error = (x - expected).norm() / expected.norm();
	cout << "Errore relativo: " << error << endl;
}





int main()
{
	//Sistemi lineari dati
	Matrix2d A1, A2, A3;
	Vector2d b1, b2, b3, expected;
	//Inizializzazione del primo sistema
	A1 << 5.547001962252291e-01, -3.770900990025203e-02, 
	8.320502943378437e-01, -9.992887623566787e-01;
	b1 << -5.169911863249772e-01, 1.672384680188350e-01;
	
	//Inizializzazione del secondo sistema
	A2 << 5.547001962252291e-01, -5.540607316466765e-01,
	8.320502943378437e-01, -8.324762492991313e-01;
    b2 << -6.394645785530173e-04, 4.259549612877223e-04;
	
	//Inizializzazione del terzo sistema
	A3 << 5.547001962252291e-01, -5.547001955851905e-01,
	8.320502943378437e-01, -8.320502947645361e-01;
	b3 << -6.400391328043042e-10, 4.266924591433963e-10;
	
	expected << -1.0, -1.0; //Soluzione attesa
	
	//Risoluzione del primo sistema
	cout << "Sistema 1: " << endl;
	x1_PALU = solveWithPALU(A1, b1);
	calculateError(x1_PALU, expected);
	x1_QR = solveWithQR(A1, b1);
	calculateError(x1_QR, expected);
	
	
	//Risoluzione del secondo sistema
	cout << "Sistema 2: " << endl;
	x2_PALU = solveWithPALU(A2, b2);
	calculateError(x2_PALU, expected);
	x2_QR = solveWithQR(A2, b2);
	calculateError(x2_QR, expected);



	//Risoluzione del terzo sistema
	cout << "Sistema 3: " << solveWithPALU(A3, b3);
	calculateError(x3_PALU, expected);
	x3_QR = solveWithQR(A3, b3);
	calculateError(x3_QR, expected);
		
    return 0;
}
