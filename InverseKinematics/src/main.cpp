
/*
 * Project: Inverse Kinematics
 *			Bachelor's Thesis at the University of Ottawa
 * Author:	Florian Hermes
 *
 * The project Inverse Kinematics implements the closed-form solutions by L. Wang et al.
 * The joint values are computed in an algebraic way.
 * In order to compute the joint values, the table-to-end-effector transformation matrix has to be calculated first.
 * The transformation matrix contains several transformations to align the coordinate systems,
 * and the rotations and translation of the desired view, selected in the desired view application.
 *
*/


#include <stdio.h>
#include <math.h>
#include <pcl\common\eigen.h>


/*
 * Rounds the result of a calculation with cos/sin and avoids negative zeros.
 *
*/
double check(double x) {
	return roundf(x * 1000000) / 1000000 +0.0;
}


/*
* Rounds the result and avoids negative zeros.
*
*/
double checkTrans(double x) {
	return roundf(x * 1) / 1 + 0.0;
}


/*
 * Computes the table-to-end-effector transformation matrix.
 * Parameter:	PA = primary Angle, SA = secondary angle, px,py,pz = Translations x,y,z from the desired view application
 *				and some parameters of the C-arm setup, which are required to compute the transformation matrix
 *
*/
Eigen::Matrix4d getTransformationMatrix(double PA, double SA, double px, double py, double pz, double l2, double a2, double D3) {

	// Declaration of Matrices, Vectors and Variables
	Eigen::Matrix4d tableToXRayMatrix = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d endEffectorToXRayMatrix = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d dVMatrix = Eigen::Matrix4d::Identity();

	Eigen::Matrix4d primAngleMatrix = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d secoAngleMatrix = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();

	Eigen::Matrix4d tabletoEndEffectorMatrix = Eigen::Matrix4d::Identity();

	float transY = l2 - D3;


	// Transfromations of Coordinate Systems

	// endEffectorToXRayMatrix = ( TransY6(a2)*Rotx6(180)*RotY6(90) )
	Eigen::Matrix4d transY6 = Eigen::Matrix4d::Identity();
	transY6(1, 3) = a2;
	Eigen::Matrix4d rotX6 = Eigen::Matrix4d::Identity();
	double b1 = 180 * M_PI / 180;
	rotX6(1, 1) = check(cos(b1));
	rotX6(1, 2) = check(-sin(b1));
	rotX6(2, 1) = check(sin(b1));
	rotX6(2, 2) = check(cos(b1));
	Eigen::Matrix4d rotY6 = Eigen::Matrix4d::Identity();
	double b2 = 90 * M_PI / 180;
	rotY6(0, 0) = check(cos(b2));
	rotY6(0, 2) = check(sin(b2));
	rotY6(2, 0) = check(-sin(b2));
	rotY6(2, 2) = check(cos(b2));

	endEffectorToXRayMatrix = (transY6 * rotX6 * rotY6);

	// tableToXRayMatrix = TransY0(transY) * RotX0(90) * RotZ0(90)
	Eigen::Matrix4d transY0 = Eigen::Matrix4d::Identity();
	transY0(1, 3) = transY;
	double a = 90 * M_PI / 180;
	Eigen::Matrix4d rotX0 = Eigen::Matrix4d::Identity();
	rotX0(1, 1) = check(cos(a));
	rotX0(1, 2) = check(-sin(a));
	rotX0(2, 1) = check(sin(a));
	rotX0(2, 2) = check(cos(a));
	Eigen::Matrix4d rotZ0 = Eigen::Matrix4d::Identity();
	rotZ0(0, 0) = check(cos(a));
	rotZ0(0, 1) = check(-sin(a));
	rotZ0(1, 0) = check(sin(a));
	rotZ0(1, 1) = check(cos(a));

	tableToXRayMatrix = transY0 * rotX0 * rotZ0;


	// DesiredViewMatrix = Rotation and Translation of CT Data

	// Primary Angle (PA):		RotZ0 = RotXxray = Q5 (RAO <-> LAO)					{[-45, 45]}
	// Secondary Angle (SA):	RotX0 = RotYxray = Q4 (CRA <-> CAU)					{[-120, 120]} except -90/90

	double primaryAngle = PA * M_PI / 180;
	primAngleMatrix(1, 1) = check(cos(primaryAngle));
	primAngleMatrix(1, 2) = check(-sin(primaryAngle));
	primAngleMatrix(2, 1) = check(sin(primaryAngle));
	primAngleMatrix(2, 2) = check(cos(primaryAngle));

	double secondaryAngle = SA * M_PI / 180;
	secoAngleMatrix(0, 0) = check(cos(secondaryAngle));
	secoAngleMatrix(0, 2) = check(sin(secondaryAngle));
	secoAngleMatrix(2, 0) = check(-sin(secondaryAngle));
	secoAngleMatrix(2, 2) = check(cos(secondaryAngle));

	translation(0, 3) = pz;
	translation(1, 3) = -px;
	translation(2, 3) = -py;

	if ((primaryAngle == 0) || (primaryAngle != 0 && secondaryAngle == 0)) {
		dVMatrix = translation * primAngleMatrix * secoAngleMatrix;
	}
	else {	// (primaryAngle != 0 && secondaryAngle != 0)

		// Alignement of Coordinate system after rotation by PA according to Guy Shechter's work

		Eigen::Matrix3d rotPA = Eigen::Matrix3d::Identity();
		Eigen::Matrix3d rotSA = Eigen::Matrix3d::Identity();
		Eigen::Matrix3d unit = Eigen::Matrix3d::Identity();
		Eigen::Matrix3d matrix;
		Eigen::Vector3d u;

		Eigen::Vector3d vector;
		vector(1) = 1;

		rotPA(1, 1) = primAngleMatrix(1, 1);
		rotPA(1, 2) = primAngleMatrix(1, 2);
		rotPA(2, 1) = primAngleMatrix(2, 1);
		rotPA(2, 2) = primAngleMatrix(2, 2);

		Eigen::Vector3d v = rotPA.transpose() * vector;

		u = v / v.norm();

		matrix.setZero();
		matrix(0, 1) = -u(2);
		matrix(0, 2) = u(1);
		matrix(1, 0) = u(2);
		matrix(1, 2) = -u(0);
		matrix(2, 0) = -u(1);
		matrix(2, 1) = u(0);

		rotSA = (u * u.transpose()) + (cos(secondaryAngle) * ( unit - (u * u.transpose()) )) + (sin(secondaryAngle) * matrix);

		secoAngleMatrix(0, 0) = rotSA(0, 0);
		secoAngleMatrix(0, 1) = rotSA(0, 1);
		secoAngleMatrix(0, 2) = rotSA(0, 2);
		secoAngleMatrix(1, 0) = rotSA(1, 0);
		secoAngleMatrix(1, 1) = rotSA(1, 1);
		secoAngleMatrix(1, 2) = rotSA(1, 2);
		secoAngleMatrix(2, 0) = rotSA(2, 0);
		secoAngleMatrix(2, 1) = rotSA(2, 1);
		secoAngleMatrix(2, 2) = rotSA(2, 2);

		dVMatrix = translation * primAngleMatrix * secoAngleMatrix;
	}

	
	// EndeffectorToTableMatrix
	tabletoEndEffectorMatrix = (tableToXRayMatrix * dVMatrix) * endEffectorToXRayMatrix.inverse();					// * patienRegistrationMatrix!!!

	// Substract Translation of End-effector due to rotation by PA/SA around isocenter
	if (PA != 0 && SA != 0) {
		tabletoEndEffectorMatrix(0, 3) = tabletoEndEffectorMatrix(0, 3) + (check(sin(primaryAngle)) * l2);
		tabletoEndEffectorMatrix(1, 3) = tabletoEndEffectorMatrix(1, 3) - (l2 - check(cos(primaryAngle))*l2);

		tabletoEndEffectorMatrix(1, 3) = tabletoEndEffectorMatrix(1, 3) - ( (check(cos(primaryAngle))*l2) - ( (check(cos(secondaryAngle)))*(check(cos(primaryAngle))*l2) ) );
		tabletoEndEffectorMatrix(2, 3) = tabletoEndEffectorMatrix(2, 3) - ( (check(cos(primaryAngle))*l2) * check(sin(secondaryAngle)) );
	}
	else {		// ((PA != 0 && SA == 0) || (PA == 0 && SA != 0))
		tabletoEndEffectorMatrix(0, 3) = tabletoEndEffectorMatrix(0, 3) + (check(sin(primaryAngle)) * l2);
		tabletoEndEffectorMatrix(1, 3) = tabletoEndEffectorMatrix(1, 3) - (l2 - check(cos(primaryAngle)) * l2);

		tabletoEndEffectorMatrix(1, 3) = tabletoEndEffectorMatrix(1, 3) - (l2 - (check(cos(secondaryAngle)) * l2)); 
		tabletoEndEffectorMatrix(2, 3) = tabletoEndEffectorMatrix(2, 3) - ((check(sin(secondaryAngle) * l2))); 
	}

	return tabletoEndEffectorMatrix;

}


/*
 * Computes the joint values in an algebraic way, susing the closed-form solution by L. Wang et al.
 * The closed-form solutions provide one equation for each joint value Q2, Q4, Q5, d0, d1, d3.
 * As two solutions exist for trigonometrical calculations, the right result has to be computed.
 * The results are printed in the console.
*/
void computeJointValues(Eigen::Matrix4d tabletoEndEffectorMatrix, double l1, double l2, double a1, double a2, double y1, double y2, double y3, double D1, double D2, double D3) {

	// Initialisation of ShortCuts for Equations
	double cy1 = check(cos(y1));
	double cy2 = check(cos(y2));
	double cy3 = check(cos(y3));
	double sy1 = check(sin(y1));
	double sy2 = check(sin(y2));
	double sy3 = check(sin(y3));

	// Define Indexes of TransformationMatrix
	double r11 = tabletoEndEffectorMatrix(0, 0);
	double r12 = tabletoEndEffectorMatrix(0, 1);
	double r13 = tabletoEndEffectorMatrix(0, 2);
	double r21 = tabletoEndEffectorMatrix(1, 0);
	double r22 = tabletoEndEffectorMatrix(1, 1);
	double r23 = tabletoEndEffectorMatrix(1, 2);
	double r31 = tabletoEndEffectorMatrix(2, 0);
	double r32 = tabletoEndEffectorMatrix(2, 1);
	double r33 = tabletoEndEffectorMatrix(2, 2);

	double px = tabletoEndEffectorMatrix(0, 3);
	double py = tabletoEndEffectorMatrix(1, 3);
	double pz = tabletoEndEffectorMatrix(2, 3);

	// Initialisation of Joint Values
	float d0 = 0;									// Translation Joint Table Z
	float d1 = 0;									// Translation Joint Table Y 
	float d3 = 0;									// Translation Joint Table X
	float Q2 = 0;									// Rotation Joint C-Arm Wigwag
	float Q4 = 0;									// Rotation Joint C-Arm Angular
	float Q5 = 0;									// Rotation Joint C-Arm Orbital
			

	// Equations

	// Wigwag Q2 (We assume  Q2 = 0 degrees for the eqautions)
	Q2 = 0 * M_PI / 180;
	double sQ2 = check(sin(Q2));
	double cQ2 = check(cos(Q2));


	// Angular Q4
	double cQ4 = (((cy1*sy2*sy3 - cy2 * cy3) *r13) - ((sy2*cy3 + cy1 * cy2*sy3) *r23) - (sy1*sy3*r33)) / (cQ2);
	double sQ4 = ((r13*sy2) - (r23*cy2) - (cy1*sy3*cQ2*cQ4 + cy1 * cy3*sQ2*cQ4)) / (sy1);
	Q4 = check(atan2(sQ4, cQ4));

	// Restrictions Q4
	if ((Q4 * 180 / M_PI) > 0) {
		Q4 = check(Q4 - M_PI);
		int i = check((Q4 * 180 / M_PI));
		if (i == -90) {
			std::cout << "ERROR: Singularity for " << Q4 * 180 / M_PI << "!" << std::endl << std::endl;
			return;
		}

	}
	else if ((Q4 * 180 / M_PI) < 0) {
		Q4 = check(Q4 + M_PI);
		int i = check((Q4 * 180 / M_PI));
		if (i == 90) {
			std::cout << "ERROR: Singularity for " << Q4 * 180 / M_PI << "!" << std::endl << std::endl;
			return;
		}
	}


	// Orbital Q5
	double cQ5 = ((r11*sy1*sy2) - (r21*sy1*cy2) + (cy1*r31)) / (check(cos(Q4)));
	double sQ5 = ((-r12 * sy1*sy2) + (r22*sy1*cy2) - (cy1*r32)) / (check(cos(Q4)));
	Q5 = check(atan2(sQ5, cQ5));


	// Translation d3
	double Eq0 = (-cy2 * px) - (sy2*py) + D1 + (cy3 * (((l2 + a1)*sQ2*check(sin(Q5))) - ((l1 - a2)*sQ2) - ((l2 + a1)*cQ2*check(sin(Q4))*check(cos(Q5))) + (a1*cQ2*check(sin(Q4))))) + (sy3 * (((l2 + a1)*sQ2*check(sin(Q4))*check(cos(Q5))) + ((l2 + a1)*cQ2*check(sin(Q5))) - ((l1 - a2)*cQ2) - (a1*sQ2*check(sin(Q4)))));
	 
	d3 = (Eq0) / (cy3*sQ2 + sy3 * cQ2);


	// Translation d1
	double Eq1 = (sy2*px) - (cy2*py) + (cy1*sy3* (((l2 + a1)*sQ2*check(sin(Q5))) - ((l1 - a2)*sQ2) - (sQ2*d3) - ((l2 + a1)*cQ2*check(sin(Q4))*check(cos(Q5))) + (a1*cQ2*check(sin(Q4))))) - (cy1*cy3* (((l2 + a1)*sQ2*check(sin(Q4))*check(cos(Q5))) + ((l2 + a1)*cQ2*check(sin(Q5))) - ((l1 - a2)*cQ2) - (cQ2*d3) - (a1*sQ2*check(sin(Q4)))));

	d1 = (Eq1 / sy1) + ((l2 + a1)*check(cos(Q4))*check(cos(Q5))) - (a1*check(cos(Q4)));			// -D3


	// Translation d0
	double Eq2 = D2 + (cy1* (d1 + D3 - ((l2 + a1)*check(cos(Q4))*check(cos(Q5))))) + (sy1*sy3* (((l2 + a1)*sQ2*check(sin(Q5))) - ((l1 - a2)*sQ2) - (sQ2*d3) - ((l2 + a1)*cQ2*check(sin(Q4))*check(cos(Q5))) + (a1*cQ2*check(sin(Q4))))) + (sy1*cy3* ((-(l2 + a1)*sQ2*check(sin(Q4))*check(cos(Q5))) - ((l2 + a1)*cQ2*check(sin(Q5))) + ((l1 - a2)*cQ2) + (cQ2*d3) + (a1*sQ2*check(sin(Q4)))));

	d0 = + pz - Eq2;


	// Results
	std::cout << "C-arm Joint Values" << std::endl;
	std::cout << "Rotations:	";
	std::cout << "Q2: " << Q2 * 180 / M_PI << std::endl;
	std::cout << "		Q5: " << Q5 * 180 / M_PI << std::endl;
	std::cout << "		Q4: " << Q4 * 180 / M_PI << std::endl;
	std::cout << "Translations:	";
	std::cout << "d3: " << checkTrans(d3) << std::endl;
	std::cout << "		d1: " << checkTrans(-d1) << std::endl;
	std::cout << "		d0: " << checkTrans(d0) << std::endl << std::endl;

}


/*
 * Initializes all the parameters of the C-arm setup and calls the implemented functions.
 *
*/
void main() {

	// Declaration of Functions
	double check(double x);
	double checkTrans(double x);
	Eigen::Matrix4d getTransformationMatrix(double PA, double SA, double px, double py, double pz, double l2, double a2, double D3);
	void computeJointValues(Eigen::Matrix4d tabletoEndEffectorMatrix, double l1, double l2, double a1, double a2, double y1, double y2, double y3, double D1, double D2, double D3);


	// Initialisation of C-Arm Systems' Parameters	   (Distances in mm)		
	double l1 = 0;									// length_offset
	double l2 = 765;								// orbital_offset
	double a1 = 0;									// read out of CT Header file - where beam center was in CT scan!!! // mostly 45
	double a2 = 0;

	double y1 = 90 * M_PI / 180;					// Ang(Z0, Z1)
	double y2 = 0 * M_PI / 180;						// Ang(X0, X1)
	double y3 = 90 * M_PI / 180;					// Ang(X1, X2)
	double D1 = 0;									// Dist(Z0, Z1)
	double D2 = 0;									// Dist(X0, X1)
	double D3 = 0;									// Dist(X1, X2)


	// Values input
	double PA = -45;									// Primary Angle (from desired views UI)
	double SA =	0;									// Secondary Angle (from desired views UI)
	double px = 10;									// Translations x,y,z (from desired views UI)
	double py = 20;
	double pz = 120;


	// Transformation Matrix
	Eigen::Matrix4d tabletoEndEffectorMatrix = getTransformationMatrix(PA, SA, px, py, pz, l2, a2, D3);
	//std::cout << "0T6: Table to End-effector" << std::endl << tabletoEndEffectorMatrix << std::endl << std::endl;

	computeJointValues(tabletoEndEffectorMatrix, l1, l2, a1, a2, y1, y2, y3, D1, D2, D3);

	getchar();
}