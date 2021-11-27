#ifndef FRICTION_CONTROLLER_H
#define FRICTION_CONTROLLER_H
#pragma once
#include <KDetailedException.h>
#include <BaseCyclicClientRpc.h> 
#include <BaseClientRpc.h> 
#include <ActuatorConfigClientRpc.h> 
#include <eigen3/Eigen/Core>
#include <friction_observer.hpp>
namespace k_api = Kinova::Api;

class friction_controller  
{
	private:

	public:
		friction_controller();
		~friction_controller();
		bool example_cyclic_torque_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config);
		int enforce_loop_frequency(const int dt);
};
#endif