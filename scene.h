#pragma once

#include "ccd.h"
#include "volInt.h"
#include <vector>

void support(const void *_obj, const ccd_vec3_t *_d, ccd_vec3_t *_p);
void stub_dir(const void *obj1, const void *obj2, ccd_vec3_t *dir);
void center(const void *_obj, ccd_vec3_t *dir);

struct Impulse {
	Eigen::RowVector3d position;
	Eigen::RowVector3d direction;
};

// the class the contains each individual rigid objects and their functionality
class Mesh
{
  public:
	Eigen::MatrixXd
		origV; // original vertex positions, where COM=(0.0,0.0,0.0) - never change this!
	Eigen::MatrixXd currV; // current vertex position
	Eigen::MatrixXi F;	 // faces of the tet mesh
	Eigen::MatrixXi T;	 // Tets in the tet mesh

	Eigen::VectorXi boundTets; // indices (from T) of just the boundary tets, for collision

	// position of object in space. We must always have that currV = QRot(origV, orientation)+ COM
	Eigen::RowVector4d orientation; // current orientation
	Eigen::RowVector3d COM;			// current center of mass
	Eigen::Matrix3d invIT; // Original *inverse* inertia tensor around the COM, defined in the rest
						   // state to the object (so to the canonical world system)

	Eigen::VectorXd tetVolumes; //|T|x1 tetrahedra volumes
	Eigen::VectorXd invMasses;  //|T|x1 tetrahedra *inverse* masses

	// kinematics
	bool isFixed;					// is the object immobile
	double totalMass;				// sum(1/invMass)
	double totalVolume;				// volume in space
	Eigen::RowVector3d comVelocity; // the linear velocity of the center of mass
	Eigen::RowVector3d angVelocity; // the angular velocity of the object.

	// dynamics
	std::vector<Impulse> currImpulses; // current list of impulses, updated by collision handling

	// checking collision between bounding boxes, and consequently the boundary tets if succeeds.
	// you do not need to update these functions (isBoxCollide and isCollide) unless you are doing a
	// different collision
	bool isBoxCollide(const Mesh &m);

	bool isCollide(const Mesh &m, double &depth, Eigen::RowVector3d &intNormal,
				   Eigen::RowVector3d &intPosition);

	// return the current inverted inertia tensor around the current COM. Update it by applying the
	// orientation
	Eigen::Matrix3d getCurrInvInertiaTensor();

	// Update the current position and orientation by integrating the linear and angular velocities,
	// and update currV accordingly You need to modify this according to its purpose
	void updatePosition(double timeStep);

	// Updating velocity *instantaneously*. i.e., not integration from acceleration, but as a result
	// of a collision impulse from the "impulses" list You need to modify this for that purpose.
	void updateImpulseVelocities();

	Eigen::RowVector3d initStaticProperties(const double density);

	// Updating the linear and angular velocities of the object
	// You need to modify this to integrate from acceleration in the field (basically gravity)
	void updateVelocity(double timeStep, const double dragCoeff);

	// the full integration for the time step (velocity + position)
	// You need to modify this if you are changing the integration
	void integrate(double timeStep, const double dragCoeff);

	Mesh(const Eigen::MatrixXd &_V, const Eigen::MatrixXi &_F, const Eigen::MatrixXi &_T,
		 const double density, const bool _isFixed, const Eigen::RowVector3d &_COM,
		 const Eigen::RowVector4d &_orientation);
};

// This class contains the entire scene operations, and the engine time loop.
class Scene
{
  public:
	double currTime;
	int numFullV, numFullT;
	std::vector<Mesh> meshes;

	// adding an objects. You do not need to update this generally
	void addMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, const Eigen::MatrixXi &T,
				 const double density, const bool isFixed, const Eigen::RowVector3d &COM,
				 const Eigen::RowVector4d &orientation)
	{
		Mesh m(V, F, T, density, isFixed, COM, orientation);
		meshes.push_back(m);
	}

	/*********************************************************************
	 This function handles a collision between objects ro1 and ro2 when found, by assigning impulses
	 to both objects. Input: RigidObjects m1, m2 depth: the depth of penetration contactNormal: the
	 normal of the conact measured m1->m2 penPosition: a point on m2 such that if m2 <= m2 +
	 depth*contactNormal, then penPosition+depth*contactNormal is the common contact point CRCoeff:
	 the coefficient of restitution
	 *********************************************************************/
	void handleCollision(Mesh &m1, Mesh &m2, const double &depth,
						 const Eigen::RowVector3d &contactNormal,
						 const Eigen::RowVector3d &penPosition, const double CRCoeff);

	/*********************************************************************
	 This function handles a single time step by:
	 1. Integrating velocities, positions, and orientations by the timeStep
	 2. detecting and handling collisions with the coefficient of restitutation CRCoeff
	 3. updating the visual scene in fullV and fullT
	 *********************************************************************/
	void updateScene(double timeStep, double CRCoeff, const double dragCoeff);

	// loading a scene from the scene .txt files
	// you do not need to update this function
	bool loadScene(const std::string dataFolder, const std::string sceneFileName);
};
