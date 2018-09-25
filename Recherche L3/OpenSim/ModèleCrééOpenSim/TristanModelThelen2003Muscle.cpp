// States 0 and 1 are defined in the base class, Thelen2003Muscle.
const int TristanThelen2003Muscle::STATE_ACTIVE_MOTOR_UNITS = 2;
const int TristanThelen2003Muscle::STATE_FATIGUED_MOTOR_UNITS = 3;

void TristanThelen2003Muscle::setNull()
{
	setType("LiuThelen2003Muscle");
	setNumStateVariables(4);
	_stateVariableSuffixes[STATE_ACTIVE_MOTOR_UNITS] = "active_motor_units";
	_stateVariableSuffixes[STATE_FATIGUED_MOTOR_UNITS] = "fatigued_motor_units";
}

void TristanThelen2003Muscle::setupProperties()
{
	_fatigueFactorProp.setName("fatigue_factor");
	_fatigueFactorProp.setValue(0.0);
	_fatigueFactorProp.setComment("percentage of active motor units that fatigue in unit time");
	_propertySet.append(&_fatigueFactorProp, "Parameters");
	_recoveryFactorProp.setName("recovery_factor");
	_recoveryFactorProp.setValue(0.0);
	_recoveryFactorProp.setComment("percentage of fatigued motor units that recover in unit time");
	_propertySet.append(&_recoveryFactorProp, "Parameters");
}

void TristanThelen2003Muscle::equilibrate(SimTK::State& state) const
{
	// Reasonable initial activation value
	setActivation(state, 0.01);
	setFiberLength(state, getOptimalFiberLength());
	setActiveMotorUnits(state, 0.0);
	setFatiguedMotorUnits(state, 0.0);
	_model->getMultibodySystem().realize(state, SimTK::Stage::Velocity);

	// Compute isometric force to get starting value of _fiberLength.
	computeEquilibrium(state);
}

void TristanThelen2003Muscle::computeEquilibrium(SimTK::State& s) const
{
	double force = computeIsometricForce(s, getActivation(s));
}

double TristanThelen2003Muscle::computeIsometricForce(SimTK::State& s, double aActivation) const
{
	if (_optimalFiberLength < ROUNDOFF_ERROR) {
		return 0.0;
	}

	// This muscle model includes two fatigue states, so this function
	// assumes that t=infinity in order to compute the [steady-state]
	// isometric force. When t=infinity, the number of active motor
	// units is independent of the activation level (as long as activation
	// > _recoveryFactor / (_fatigueFactor + _recoveryFactor)). So the
	// passed-in activation is not used in this function (unless the fatigue
	// and recovery factors are both zero which means there is no fatigue).
	if ((_fatigueFactor + _recoveryFactor > 0.0) && (aActivation >=
		_recoveryFactor / (_fatigueFactor + _recoveryFactor))) {
		setActiveMotorUnits(s, _recoveryFactor / (_fatigueFactor + _recoveryFactor));
		setFatiguedMotorUnits(s, _fatigueFactor / (_fatigueFactor + _recoveryFactor));
	}

	else {
		setActiveMotorUnits(s, aActivation);
		setFatiguedMotorUnits(s, 0.0);
	}

	aActivation = getActiveMotorUnits(s);

	// Now you can call the base class's function with the steady-state
	// activation.
	return Thelen2003Muscle::computeIsometricForce(s, aActivation);
}