#include "scene.h"

#include "auxfunctions.h"

#include <fstream>
#include <igl/readMESH.h>

using namespace Eigen;
using namespace std;

/**************************************************************************
 * https://en.wikipedia.org/wiki/List_of_equations_in_classical_mechanics *
 **************************************************************************/

/********** Mesh Class ************/

// checking collision between bounding boxes, and consequently the boundary tets if succeeds.
// you do not need to update these functions (isBoxCollide and isCollide) unless you are doing a
// different collision
bool Mesh::isBoxCollide(const Mesh &m)
{
	RowVector3d VMin1 = currV.colwise().minCoeff();
	RowVector3d VMax1 = currV.colwise().maxCoeff();
	RowVector3d VMin2 = m.currV.colwise().minCoeff();
	RowVector3d VMax2 = m.currV.colwise().maxCoeff();

	// checking all axes for non-intersection of the dimensional interval
	for (int i = 0; i < 3; i++)
		if ((VMax1(i) < VMin2(i)) || (VMax2(i) < VMin1(i)))
			return false;

	return true; // all dimensional intervals are overlapping = intersection
}

bool Mesh::isCollide(const Mesh &m, double &depth, RowVector3d &intNormal, RowVector3d &intPosition)
{

	if ((isFixed && m.isFixed)) // collision does nothing
		return false;

	// collision between bounding boxes
	if (!isBoxCollide(m))
		return false;

	// otherwise, full test
	ccd_t ccd;
	CCD_INIT(&ccd);
	ccd.support1 = support; // support function for first object
	ccd.support2 = support; // support function for second object
	ccd.center1 = center;
	ccd.center2 = center;

	ccd.first_dir = stub_dir;
	ccd.max_iterations = 100; // maximal number of iterations

	void *obj1 = (void *)this;
	void *obj2 = (void *)&m;

	ccd_real_t _depth;
	ccd_vec3_t dir, pos;

	int nonintersect = ccdMPRPenetration(obj1, obj2, &ccd, &_depth, &dir, &pos);

	if (nonintersect)
		return false;

	for (int k = 0; k < 3; k++)
	{
		intNormal(k) = dir.v[k];
		intPosition(k) = pos.v[k];
	}

	depth = _depth;
	intPosition -= depth * intNormal / 2.0;

	// Vector3d p1=intPosition+depth*intNormal;
	// Vector3d p2=intPosition;
	// cout<<"intPosition: "<<intPosition<<endl;

	// cout<<"depth: "<<depth<<endl;
	// cout<<"After ccdGJKIntersect"<<endl;

	// return !nonintersect;

	return true;
}

// return the current inverted inertia tensor around the current COM. Update it by applying the
// orientation
Matrix3d Mesh::getCurrInvInertiaTensor()
{
	Matrix3d R = Q2RotMatrix(orientation);
	/***************
	 *     TODO     *
	 ****************/
	// return Matrix3d::Identity(3,3);  //change this to your result
#if 1
	return R.transpose() * invIT * R;
#else
	return R * invIT * R.transpose(); // Answer
#endif
}

// Update the current position and orientation by integrating the linear and angular velocities, and
// update currV accordingly
// You need to modify this according to its purpose
void Mesh::updatePosition(double timeStep)
{
	// just forward Euler now
	if (isFixed)
		return; // a fixed object is immobile

	/***************
	 *     TODO     *
	 ****************/

	// Answer
	// TODO: look at more fun time integration

	// translational velocity
	COM += comVelocity * timeStep;

	// angular velocity
	RowVector4d angQuaternion;
	angQuaternion << 0.0, 0.5 * angVelocity * timeStep;
	orientation += QMult(angQuaternion, orientation);
	// renormalizing for stability
	orientation.normalize();
	// !Answer
	for (int i = 0; i < currV.rows(); i++)
		currV.row(i) << QRot(origV.row(i), orientation) + COM;
}

// Updating velocity *instantaneously*. i.e., not integration from acceleration, but as a result of
// a collision impulse from the "impulses" list You need to modify this for that purpose.
void Mesh::updateImpulseVelocities()
{

	if (isFixed)
	{
		comVelocity.setZero();
		currImpulses.clear();
		angVelocity.setZero();
		return;
	}

	// update linear and angular velocity according to all impulses
	/***************
	 *     TODO     *
	 ****************/

	// Answer
	for (const auto &impulse : currImpulses)
	{
		// Note: impulse is corrected when inserted so no weird minus shit here.

		// Radius vector
		RowVector3d r = impulse.position - COM;
		// torque impulse vector: dL = r x jn
		RowVector3d torqueImpulse = r.cross(impulse.direction);
		// (torque impulse -> instantaneous change in angular momentum, i.e. in angular velocity)

		// Linear accelaration: a = jn / m
		comVelocity += impulse.direction / totalMass;
		// Angluar accelaration: a =  I^{-1} * dL^{T}
		angVelocity += getCurrInvInertiaTensor() * torqueImpulse.transpose();
	}
	currImpulses.clear();
	// !Answer
}

RowVector3d Mesh::initStaticProperties(const double density)
{
	// TODO: compute tet volumes and allocate to vertices
	tetVolumes.conservativeResize(T.rows());

	RowVector3d naturalCOM;
	naturalCOM.setZero();
	Matrix3d IT;
	IT.setZero();
	for (int i = 0; i < T.rows(); i++)
	{
		Vector3d e01 = origV.row(T(i, 1)) - origV.row(T(i, 0));
		Vector3d e02 = origV.row(T(i, 2)) - origV.row(T(i, 0));
		Vector3d e03 = origV.row(T(i, 3)) - origV.row(T(i, 0));
		Vector3d tetCentroid =
			(origV.row(T(i, 0)) + origV.row(T(i, 1)) + origV.row(T(i, 2)) + origV.row(T(i, 3))) /
			4.0;
		tetVolumes(i) = abs(e01.dot(e02.cross(e03))) / 6.0;

		naturalCOM += tetVolumes(i) * tetCentroid;
	}

	totalVolume = tetVolumes.sum();
	totalMass = density * totalVolume;
	naturalCOM.array() /= totalVolume;

	// computing inertia tensor
	for (int i = 0; i < T.rows(); i++)
	{
		RowVector4d xvec;
		xvec << origV(T(i, 0), 0) - naturalCOM(0), origV(T(i, 1), 0) - naturalCOM(0),
			origV(T(i, 2), 0) - naturalCOM(0), origV(T(i, 3), 0) - naturalCOM(0);
		RowVector4d yvec;
		yvec << origV(T(i, 0), 1) - naturalCOM(1), origV(T(i, 1), 1) - naturalCOM(1),
			origV(T(i, 2), 1) - naturalCOM(1), origV(T(i, 3), 1) - naturalCOM(1);
		RowVector4d zvec;
		zvec << origV(T(i, 0), 2) - naturalCOM(2), origV(T(i, 1), 2) - naturalCOM(2),
			origV(T(i, 2), 2) - naturalCOM(2), origV(T(i, 3), 2) - naturalCOM(2);

		double I00, I11, I22, I12, I21, I01, I10, I02, I20;
		Matrix4d sumMat = Matrix4d::Constant(1.0) + Matrix4d::Identity();
		I00 = density * 6 * tetVolumes(i) *
			  (yvec * sumMat * yvec.transpose() + zvec * sumMat * zvec.transpose()).sum() / 120.0;
		I11 = density * 6 * tetVolumes(i) *
			  (xvec * sumMat * xvec.transpose() + zvec * sumMat * zvec.transpose()).sum() / 120.0;
		I22 = density * 6 * tetVolumes(i) *
			  (xvec * sumMat * xvec.transpose() + yvec * sumMat * yvec.transpose()).sum() / 120.0;
		I12 = I21 = -density * 6 * tetVolumes(i) * (yvec * sumMat * zvec.transpose()).sum() / 120.0;
		I10 = I01 = -density * 6 * tetVolumes(i) * (xvec * sumMat * zvec.transpose()).sum() / 120.0;
		I20 = I02 = -density * 6 * tetVolumes(i) * (xvec * sumMat * yvec.transpose()).sum() / 120.0;

		Matrix3d currIT;
		currIT << I00, I01, I02, I10, I11, I12, I20, I21, I22;

		IT += currIT;
	}
	invIT = IT.inverse();

	// compare to function
	// double massCompare;
	// RowVector3d COMcompare;
	// Matrix3d invITcompare;
	// getCOMandInvIT(origV, F, density, massCompare, COMcompare, invITcompare);
	// cout<<"massCompare-totalMass"<<massCompare-totalMass<<endl;
	// cout<<"COMcompare-naturalCOM"<<COMcompare-naturalCOM<<endl;
	// cout<<"invITcompare-invIT"<<invITcompare-invIT<<endl;

	return naturalCOM;
}

// Updating the linear and angular velocities of the object
// You need to modify this to integrate from acceleration in the field (basically gravity)
void Mesh::updateVelocity(double timeStep, const double dragCoeff)
{

	if (isFixed)
		return;

	// integrating external forces (only gravity)
	RowVector3d gravity;
	gravity << 0, -9.8, 0.0;
	comVelocity += (gravity - dragCoeff / totalMass * comVelocity) * timeStep;
	angVelocity -= dragCoeff * getCurrInvInertiaTensor() * angVelocity.transpose() * timeStep;
}

// the full integration for the time step (velocity + position)
// You need to modify this if you are changing the integration
void Mesh::integrate(double timeStep, const double dragCoeff)
{
	updateVelocity(timeStep, dragCoeff);
	updatePosition(timeStep);
}

Mesh::Mesh(const MatrixXd &_V, const MatrixXi &_F, const MatrixXi &_T, const double density,
		   const bool _isFixed, const RowVector3d &_COM, const RowVector4d &_orientation)
{
	origV = _V;
	F = _F;
	T = _T;
	isFixed = _isFixed;
	COM = _COM;
	orientation = _orientation;
	comVelocity.setZero();
	angVelocity.setZero();

	RowVector3d naturalCOM; // by the geometry of the object

	// initializes the original geometric properties (COM + IT) of the object
	naturalCOM = initStaticProperties(density);

	origV.rowwise() -=
		naturalCOM; // removing the natural COM of the OFF file (natural COM is never used again)

	currV.resize(origV.rows(), origV.cols());
	for (int i = 0; i < currV.rows(); i++)
		currV.row(i) << QRot(origV.row(i), orientation) + COM;

	VectorXi boundVMask(origV.rows());
	boundVMask.setZero();
	for (int i = 0; i < F.rows(); i++)
		for (int j = 0; j < 3; j++)
			boundVMask(F(i, j)) = 1;

	// cout<<"boundVMask.sum(): "<<boundVMask.sum()<<endl;

	vector<int> boundTList;
	for (int i = 0; i < T.rows(); i++)
	{
		int incidence = 0;
		for (int j = 0; j < 4; j++)
			incidence += boundVMask(T(i, j));
		if (incidence > 2)
			boundTList.push_back(i);
	}

	boundTets.resize(boundTList.size());
	for (int i = 0; i < boundTets.size(); i++)
		boundTets(i) = boundTList[i];
}

/************ Scene class **********************/

/*********************************************************************
   This function handles a collision between objects ro1 and ro2 when found, by assigning impulses
   to both objects. Input: RigidObjects m1, m2 depth: the depth of penetration contactNormal: the
   normal of the conact measured m1->m2 penPosition: a point on m2 such that if m2 <= m2 +
   depth*contactNormal, then penPosition+depth*contactNormal is the common contact point CRCoeff:
   the coefficient of restitution
   *********************************************************************/
void Scene::handleCollision(Mesh &m1, Mesh &m2, const double &depth,
							const RowVector3d &contactNormal, const RowVector3d &penPosition,
							const double CRCoeff)
{
	// cout<<"handleCollision begin"<<endl;

	// Interpretation resolution: move each object by inverse mass weighting, unless either is
	// fixed, and then move the other. Remember to respect the direction of contactNormal and update
	// penPosition accordingly.
	RowVector3d contactPosition = penPosition;
	if (m1.isFixed)
	{
		/***************
		 *     TODO     *
		 ****************/
		// Answer

		m2.currV.rowwise() += depth * contactNormal; // Correct mesh vertices
		m2.COM += depth * contactNormal;			 // Correct center of mass
		contactPosition += depth * contactNormal;	// Correct contact point

		// !Answer
	}
	else if (m2.isFixed)
	{
		/***************
		 *     TODO     *
		 ****************/
		// Answer

		m1.currV.rowwise() -= depth * contactNormal; // Correct mesh vertices
		m1.COM -= depth * contactNormal;			 // Correct center of mass
		contactPosition -= depth * contactNormal;	// Correct contact point

		// !Answer
	}
	else
	{ // inverse mass weighting
		/***************
		 *     TODO     *
		 ****************/
		// Answer

		double combinedMass = m1.totalMass + m2.totalMass; // Combined mass of objects
		double weight2 = m1.totalMass / combinedMass;	  // Weighting for object 2
		double weight1 = m2.totalMass / combinedMass;	  // Weighting for object 1

		m1.currV.rowwise() -= weight1 * depth * contactNormal; // Correct mesh vertices object 1
		m1.COM -= weight1 * depth * contactNormal;			   // Correct center of mass object 1
		m2.currV.rowwise() += weight2 * depth * contactNormal; // Correct mesh vertices object 2
		m2.COM += weight2 * depth * contactNormal;			   // Correct center of mass object 2

		contactPosition += weight2 * depth * contactNormal; // Correct contact position

		// !Answer
	}

	// Create impulses and push them into ro1.impulses and ro2.impulses.
	/***************
	 *     TODO     *
	 ****************/
	// RowVector3d impulse=RowVector3d::Zero();  //change this to your result

	// Answer

	RowVector3d r1 = contactPosition - m1.COM; // Radius Vector object 1
	RowVector3d r2 = contactPosition - m2.COM; // Radius Vector object 2

	// Compute the velocities *in the contact points* of both meshes:
	// Full velocity (anglular and linear) object 1
	RowVector3d fullv1 = m1.comVelocity + m1.angVelocity.cross(r1);
	// Full velocity (anglular and linear) object 2
	RowVector3d fullv2 = m2.comVelocity + m2.angVelocity.cross(r2);
	// Closing velocity relative to contactnormal: (v1 - v2) * n
	double closingVcosN = contactNormal.dot(fullv2 - fullv1);

	RowVector3d r1crossn = r1.cross(contactNormal); // r1 x n
	RowVector3d r2crossn = r2.cross(contactNormal); // r2 x n

	// The resistance to rotation (of both objects) away from the radius and contact normal:
	// 1.0 / augmented masses and inertia =
	// (1/m1 + 1/m2) + [(r1 x n)^T I1^{-1} (r1 x n) + (r2 x n)^T I2^{-1} (r2 x n)]
	double invAugMasses = 1.0 / ((1.0 / m1.totalMass) + (1.0 / m2.totalMass) +
								 r2crossn * m2.getCurrInvInertiaTensor() * r2crossn.transpose() +
								 r1crossn * m1.getCurrInvInertiaTensor() * r1crossn.transpose());

	// impulse jn =	((-(1 + CR) * (closingVcosN)) / augmasses) n
	RowVector3d impulse = -(1 + CRCoeff) * closingVcosN * invAugMasses * contactNormal;
	// !Answer

	// cout << "impulse: " << impulse << endl;
	if (impulse.norm() > 10e-6)
	{
		m1.currImpulses.push_back(Impulse{contactPosition, -impulse});
		m2.currImpulses.push_back(Impulse{contactPosition, impulse});
	}

	// cout<<"handleCollision end"<<endl;

	// updating velocities according to impulses
	m1.updateImpulseVelocities();
	m2.updateImpulseVelocities();
}

/*********************************************************************
   This function handles a single time step by:
   1. Integrating velocities, positions, and orientations by the timeStep
   2. detecting and handling collisions with the coefficient of restitutation CRCoeff
   3. updating the visual scene in fullV and fullT
   *********************************************************************/
void Scene::updateScene(double timeStep, double CRCoeff, const double dragCoeff)
{

	// integrating velocity, position and orientation from forces and previous states
	for (int i = 0; i < meshes.size(); i++)
		meshes[i].integrate(timeStep, dragCoeff);

	// detecting and handling collisions when found
	// This is done exhaustively: checking every two objects in the scene.
	double depth;
	RowVector3d contactNormal, penPosition;
	for (int i = 0; i < meshes.size(); i++)
		for (int j = i + 1; j < meshes.size(); j++)
			if (meshes[i].isCollide(meshes[j], depth, contactNormal, penPosition))
				handleCollision(meshes[i], meshes[j], depth, contactNormal, penPosition, CRCoeff);

	currTime += timeStep;
}

// loading a scene from the scene .txt files
// you do not need to update this function
bool Scene::loadScene(const string dataFolder, const string sceneFileName)
{

	ifstream sceneFileHandle;
	sceneFileHandle.open(dataFolder + string("/") + sceneFileName);
	if (!sceneFileHandle.is_open())
		return false;
	int numofObjects;

	currTime = 0;
	sceneFileHandle >> numofObjects;
	for (int i = 0; i < numofObjects; i++)
	{
		MatrixXi objT, objF;
		MatrixXd objV;
		string MESHFileName;
		bool isFixed;
		double youngModulus, poissonRatio, density;
		RowVector3d userCOM;
		RowVector4d userOrientation;
		sceneFileHandle >> MESHFileName >> density >> youngModulus >> poissonRatio >> isFixed >>
			userCOM(0) >> userCOM(1) >> userCOM(2) >> userOrientation(0) >> userOrientation(1) >>
			userOrientation(2) >> userOrientation(3);
		userOrientation.normalize();
		igl::readMESH(dataFolder + string("/") + MESHFileName, objV, objT, objF);

		// fixing weird orientation problem
		MatrixXi tempF(objF.rows(), 3);
		tempF << objF.col(2), objF.col(1), objF.col(0);
		objF = tempF;

		addMesh(objV, objF, objT, density, isFixed, userCOM, userOrientation);
		cout << "COM: " << userCOM << endl;
		cout << "orientation: " << userOrientation << endl;
	}
	return true;
}

/*************Auxiliary functions for collision detection. Do not need updating******************/

/** Support function for libccd*/
void support(const void *_obj, const ccd_vec3_t *_d, ccd_vec3_t *_p)
{
	// assume that obj_t is user-defined structure that holds info about
	// object (in this case box: x, y, z, pos, quat - dimensions of box,
	// position and rotation)
	// cout<<"calling support"<<endl;
	Mesh *obj = (Mesh *)_obj;
	RowVector3d p;
	RowVector3d d;
	for (int i = 0; i < 3; i++)
		d(i) = _d->v[i]; // p(i)=_p->v[i];

	d.normalize();
	// cout<<"d: "<<d<<endl;

	int maxVertex = -1;
	int maxDotProd = -32767.0;
	for (int i = 0; i < obj->currV.rows(); i++)
	{
		double currDotProd = d.dot(obj->currV.row(i) - obj->COM);
		if (maxDotProd < currDotProd)
		{
			maxDotProd = currDotProd;
			// cout<<"maxDotProd: "<<maxDotProd<<endl;
			maxVertex = i;
		}
	}
	// cout<<"maxVertex: "<<maxVertex<<endl;

	for (int i = 0; i < 3; i++)
		_p->v[i] = obj->currV(maxVertex, i);

	// cout<<"end support"<<endl;
}

void stub_dir(const void *obj1, const void *obj2, ccd_vec3_t *dir)
{
	dir->v[0] = 1.0;
	dir->v[1] = 0.0;
	dir->v[2] = 0.0;
}

void center(const void *_obj, ccd_vec3_t *center)
{
	Mesh *obj = (Mesh *)_obj;
	for (int i = 0; i < 3; i++)
		center->v[i] = obj->COM(i);
}
