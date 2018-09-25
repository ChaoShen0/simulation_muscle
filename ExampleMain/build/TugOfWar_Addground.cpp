/* -------------------------------------------------------------------------- *
*                    OpenSim:  TugOfWar1_CreateModel.cpp                     *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2012 Stanford University and the Authors                *
* Author(s): Jeffrey A. Reinbolt, Ayman Habib, Ajay Seth, Jack Middleton,    *
*            Samuel R. Hamner                                                *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License"); you may    *
* not use this file except in compliance with the License. You may obtain a  *
* copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and        *
* limitations under the License.                                             *
* -------------------------------------------------------------------------- */

/*
*  Below is an example of an OpenSim application that provides its own
*  main() routine.  This application is a forward simulation of tug-of-war between two
*  muscles pulling on a block.
*/

// Author:  Jeff Reinbolt, Ayman Habib, Ajay Seth, Jack Middleton, Samuel Hamner

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;



//______________________________________________________________________________
/**
* First exercise: create a model that does nothing.
*/
int main()
{	
	try {

			// Create an OpenSim model and set its name
			Model osimModel;
			osimModel.setName("Bras_simple");
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

			OpenSim::Body* humerus = new OpenSim::Body("humerus", 3, Vec3(0,-0.1425,0), Inertia(0)); //Création de la partie Humérus
			OpenSim::Body* radius = new OpenSim::Body("radius", 2, Vec3(0,-0.115,0), Inertia(0)); //Création de la partie Radius
			OpenSim::Body* sphere = new OpenSim::Body("sphere", 2, Vec3(0), Inertia(0));
			OpenSim::ContactSphere(0.1, Vec3(0), *sphere);
			humerus->addDisplayGeometry("humerus.vtp");
			radius->addDisplayGeometry("radius.vtp");
		

			PinJoint* shoulder = new PinJoint("shoulder",
				// Parent body, location in parent, orientation in parent.
				osimModel.getGroundBody(), Vec3(0), Vec3(0),
				// Child body, location in child, orientation in child.
				*humerus, Vec3(0, 0.01, 0), Vec3(0,0,-80));

			PinJoint* rotula = new PinJoint("rotula",
				// Parent body, location in parent, orientation in parent.
				*humerus, Vec3(0, -0.285, 0.06), Vec3(0),
				// Child body, location in child, orientation in child.
				*sphere, Vec3(0, 0.01, 0), Vec3(0, 0, 0));


			PinJoint* elbow = new PinJoint("elbow",
				*sphere, Vec3(0,0.01,0), Vec3(0), *radius, Vec3(0, 0, 0), Vec3(0,0,250));

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


			muscle3->addNewPathPoint("muscle3-point",*humerus,Vec3(-0.016,-0.05,0));
			muscle3->addNewPathPoint("muscle3-point",*radius,Vec3(-0.01,-0.001,0));


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
			muscleController->prescribeControlForActuator("muscle1", new MultiplierFunction(new LinearFunction(slopeAndIntercept1),100.0));

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
			
			
			





			// Save the simulation results
			Storage statesDegrees(manager.getStateStorage());
			statesDegrees.print("biceps.sto");
			osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
			statesDegrees.setWriteSIMMHeader(true);
			statesDegrees.print("biceps.mot");

			osimModel.print("biceps.osim");

	}
	catch (OpenSim::Exception ex)
	{
		std::cout << ex.getMessage() << std::endl;
		return 1;
	}
	catch (std::exception ex)
	{
		std::cout << ex.what() << std::endl;
		return 1;
	}
	catch (...)
	{
		std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
		return 1;
	}
	

	std::cout << "OpenSim example completed successfully.\n";
	std::cin.get();
	return 0;
}
