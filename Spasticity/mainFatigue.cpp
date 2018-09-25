/* -------------------------------------------------------------------------- *
 * Tristan Venot                         *
Travail sur la spasticité
 */

//=============================================================================
//=============================================================================
#include <OpenSim/OpenSim.h>
#include "FatigableMuscle.h"
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Common/IO.h>

using namespace OpenSim;
using namespace SimTK;

//_____________________________________________________________________________
/**
 * Run a simulation of a sliding block being pulled by two muscle 
 */
int main()
{
    std::clock_t startTime = std::clock();

	try {


		// Create an OpenSim model and set its name
		Model osimModel;
		osimModel.setName("Etude_spasticité");
		// Initialize the system
		SimTK::State& si = osimModel.initSystem();
		// Get a reference to the model's ground body
		// Create the integrator and manager for the simulation.
		SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
		integrator.setAccuracy(1.0e-4);
		Manager manager(osimModel, integrator);

		// Define the initial and final simulation times
		double initialTime = 0.0;
		double finalTime = 4.0;
		// Integrate from initial time to final time
		manager.setInitialTime(initialTime);
		manager.setFinalTime(finalTime);
		std::cout << "\n\nIntegrating from " << initialTime << " to " << finalTime << std::endl;
		manager.integrate(si);








		OpenSim::Body& ground = osimModel.getGroundBody();

		// Add display geometry to the ground to visualize in the GUI
		ground.addDisplayGeometry("ground.vtp");

		OpenSim::Body* humerus = new OpenSim::Body("humerus", 3, Vec3(0, -0.1425, 0), Inertia(0)); //Création de la partie Humérus
		OpenSim::Body* radius = new OpenSim::Body("radius", 2, Vec3(0, -0.115, 0), Inertia(0)); //Création de la partie Radius
		OpenSim::Body* sphere = new OpenSim::Body("sphere", 2, Vec3(0), Inertia(0));
		OpenSim::ContactSphere(0.1, Vec3(0), *sphere);
		humerus->addDisplayGeometry("humerus.vtp");
		radius->addDisplayGeometry("radius.vtp");


		PinJoint* shoulder = new PinJoint("shoulder",
			// Parent body, location in parent, orientation in parent.
			osimModel.getGroundBody(), Vec3(0), Vec3(0),
			// Child body, location in child, orientation in child.
			*humerus, Vec3(0, 0.01, 0), Vec3(0, 0, -80));

		PinJoint* rotula = new PinJoint("rotula",
			// Parent body, location in parent, orientation in parent.
			*humerus, Vec3(0, -0.285, 0.06), Vec3(0),
			// Child body, location in child, orientation in child.
			*sphere, Vec3(0, 0.01, 0), Vec3(0, 0, 0));


		PinJoint* elbow = new PinJoint("elbow",
			*sphere, Vec3(0, 0.01, 0), Vec3(0), *radius, Vec3(0, 0, 0), Vec3(0, 0, 250));

		osimModel.addBody(humerus);
		osimModel.addBody(sphere);
		// Define the acceleration of gravity
		osimModel.setGravity(Vec3(-9.80665, 0, 0));
		osimModel.addBody(radius);
		// Save the model to a file

		double maxIsometricForce1 = 80.0, optimalFiberLength1 = 0.1, tendonSlackLength1 = 0.2, pennationAngle1 = 0.0,
			activation = 0.0001, deactivation = 1.0;
		double maxIsometricForce2 = 30.0, optimalFiberLength2 = 0.05, tendonSlackLength2 = 0.02, pennationAngle2 = 0.0;
		double maxIsometricForce3 = 30.0, optimalFiberLength3 = 0.2, tendonSlackLength3 = 0.2, pennationAngle3 = 0.0;

		// Create two new muscles
		double fatigueFactor = 0.30, recoveryFactor = 0.20;



		// Create new muscle 1 using the Shutte 1993 muscle model
		// Note: activation/deactivation parameters are set differently between the models.
		Millard2012EquilibriumMuscle *muscle1 = new
			Millard2012EquilibriumMuscle("muscle1", maxIsometricForce1,
			optimalFiberLength1, tendonSlackLength1, pennationAngle1);
		muscle1->setActivationTimeConstant(activation);
		muscle1->setDeactivationTimeConstant(deactivation);

		Millard2012EquilibriumMuscle *muscle2 = new
			Millard2012EquilibriumMuscle("muscle2", maxIsometricForce2,
			optimalFiberLength2, tendonSlackLength2, pennationAngle2);
		muscle2->setActivationTimeConstant(activation);
		muscle2->setDeactivationTimeConstant(deactivation);


		Millard2012EquilibriumMuscle *muscle3 = new
			Millard2012EquilibriumMuscle("muscle3", maxIsometricForce3,
			optimalFiberLength3, tendonSlackLength3, pennationAngle3);
		muscle2->setActivationTimeConstant(activation);
		muscle2->setDeactivationTimeConstant(deactivation);


		muscle1->addNewPathPoint("muscle1-point1", *humerus, Vec3(0.0, -0.01, 0));
		muscle1->addNewPathPoint("muscle1-point2", *radius, Vec3(0.0, -0.1, 0.04));

		muscle2->addNewPathPoint("muscle2-point1", *humerus, Vec3(0.0, -0.2, 0.03));
		muscle2->addNewPathPoint("muscle2-point2", *radius, Vec3(0.0, -0.07, 0.02));


		muscle3->addNewPathPoint("muscle3-point", *humerus, Vec3(-0.016, -0.05, 0));
		muscle3->addNewPathPoint("muscle3-point", *radius, Vec3(-0.01, -0.001, 0));


		osimModel.addForce(muscle1);
		osimModel.addForce(muscle2);
		osimModel.addForce(muscle3);

		// Create a prescribed controller that simply applies controls as function of time
		PrescribedController *muscleController = new PrescribedController();
		muscleController->setActuators(osimModel.updActuators());



		// Define linear functions for the control values for the two muscles
		Array<double> slopeAndIntercept1(0.0, 2); // array of 2 doubles
		Array<double> slopeAndIntercept2(0.0, 2);
		Array<double> slopeAndIntercept3(0.0, 2);
		// muscle1 control has slope of -1 starting 1 at t = 0
		slopeAndIntercept1[0] = -1.0 / (finalTime - initialTime); slopeAndIntercept1[1] = 1.0;

		// muscle2 control has slope of 1 starting 0.05 at t = 0
		slopeAndIntercept2[0] = 1.0 / (finalTime - initialTime); slopeAndIntercept2[1] = 0.05;

		//Muscle 3 control :

		slopeAndIntercept3[0] = 1.0 / (finalTime - initialTime); slopeAndIntercept2[1] = 1.0;
		// Set the indiviudal muscle control functions for the prescribed muscle controller
		muscleController->prescribeControlForActuator("muscle1", new MultiplierFunction(new LinearFunction(slopeAndIntercept1), 100.0));

		muscleController->prescribeControlForActuator("muscle2", new LinearFunction(slopeAndIntercept2));
		muscleController->prescribeControlForActuator("muscle3", new LinearFunction(slopeAndIntercept3));
		// Add the muscle controller to the model
		osimModel.addController(muscleController);

		// Define the initial states for the two muscles
		// Initial activation correspond to control at time=0
		muscle1->setDefaultActivation(slopeAndIntercept1[1]);
		muscle2->setDefaultActivation(slopeAndIntercept2[1]);
		muscle3->setDefaultActivation(slopeAndIntercept3[1]);
		// Fiber length

		muscle1->setDefaultFiberLength(0.1);
		muscle2->setDefaultFiberLength(0.1);
		muscle3->setDefaultFiberLength(0.5);





		double speed = muscle1 -> getLengtheningSpeed(si);
		if (speed >= 10)
		{

			osimModel.print("studychoice1.osim");

		}
		else{

		// Save the simulation results
		Storage statesDegrees(manager.getStateStorage());
		statesDegrees.print("biceps.sto");
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.setWriteSIMMHeader(true);
		statesDegrees.print("biceps.mot");

		osimModel.print("etudespasticity2.0.osim");
			}
	}
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }

    std::cout << "main() routine time = " << 1.e3*(std::clock()-startTime)/CLOCKS_PER_SEC << "ms\n";

    std::cout << "OpenSim example completed successfully.\n";
	return 0;
}
