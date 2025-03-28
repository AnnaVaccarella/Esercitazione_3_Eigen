#include <iostream>
#include <Eigen>

using namespace std;

// PALU
void solveWithPALU (const Matrix2d& A, const Vector2d& b) {
	PartialPivLU<Matrix2d> lu(A);
	Vector2d x = lu.solve(b);
	cout << "Soluzione con PALU: " << x.transpose() << endl;
}

// QR
void solveWithQR (const Matrix2d& A, const Vector2d& b) {
	HouseHolderQR<Matrix2d> qr(A);
	Vector2d x = qr.solve(b);
	cout << "Soluzione con QR: " << x.transpose() << endl;
}

// calcola errore relativo
void calculateError(const Vector2d& x, const Vector2d& expected) {
	double error = (x - expected).norm() / expected.norm();
	cout << "Errore relativo: " << error << endl;
}


