#include <OpenSim/Actuators/Thelen2003Muscle.h>
namespace OpenSim {

	class TristanThelen2003Muscle : public Thelen2003Muscle
	{
	protected:
		// the rate at which active muscle fibers become fatigued
		PropertyDbl _fatigueFactorProp;
		double &_fatigueFactor;

		// the rate at which fatigued fibers recover (become active)
		PropertyDbl _recoveryFactorProp;
		double &_recoveryFactor;
	protected:
		static const int STATE_ACTIVE_MOTOR_UNITS;
		static const int STATE_FATIGUED_MOTOR_UNITS;

	public:
		virtual void computeEquilibrium(SimTK::State& s) const;
		virtual double computeActuation(const SimTK::State& s) const;
		virtual double computeIsometricForce(SimTK::State& s, double act) const;
		virtual void equilibrate(SimTK::State& state) const;

	private:
		void setNull();
		void setupProperties();
	}