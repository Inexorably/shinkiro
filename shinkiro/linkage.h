#pragma once

#include <iostream>
#include <vector>

#include "math.h"

#include "globals.h"

#include "Eigen/Dense"

namespace shinkiro {
	//Class defining individual links of a linkage.  Assumes units are consistent.
	class Link {
		public:
			//Default constructor, primarily useful for testing so we fill with some arbitrary values.
			Link();

			double m_length;		//Link length.
			double m_radius;		//Distance along the link from the link origin to the center of mass.
			double m_inertiaCenter;	//Inertia about center.
			double m_inertiaOrigin;	//Inertia about link origin.
			double m_mass;
			double m_theta;			//Current angle in rad.
			double m_omega;			//Current angular velocity in rad/s.
			double m_alpha;			//Current angular acceleration in rad/s.
			
			//These values can either be set by user for forward dynamics or found and set by inverse dynamics function.
			double m_forceX;
			double m_forceY;
			double m_torque;		//Ccw positive.

			//Some useful functions for simplifying calculation code.
			double f_cta() const;		//Returns cos(theta)*alpha.
			double f_sta() const;		//Returns sin(theta)*alpha.
			double f_ctw2() const;		//Returns cos(theta)*omega^2.
			double f_stw2() const;		//Returns sin(theta)*omega^2.
	};

	class Linkage {
		public:
			std::vector<Link> m_links;

			//Use inverse dynamics to calculate the forces and moments for a three link planar bipedal / inverted pendulum model.
			//Assumes foot is massless and remains grounded for period of analysis.
			//Returns a vector of the forces and torques of order [Fx1; Fx2; Fy1; Fy2; T1; T2; Fx3; Fy3; T3].
			Eigen::VectorXd f_inverseDynamics() const;

			//Use forward dynamics to find the angular accelerations resulting from some given forces and torques.
			//Assumes foot is massless and remains grounded for period of analysis.
			//Returns a vector of angular accelerations of order [alpha1; alpha2; alpha3].
			Eigen::VectorXd f_forwardDynamicsFull() const;

			//Use forward dynamics to find the angular accelerations resulting from torques only.
			//Assumes foot is massless and remains grounded for period of analysis.
			//Returns a vector of angular accelerations of order [alpha1; alpha2; alpha3].
			Eigen::VectorXd f_forwardDynamicsTorques(Eigen::VectorXd torques) const;

			//Steps forward in the current linkage applying some torques.
			//Returns a copy of the result.  NOTE: INTEGRATES CURRENT LINKAGE OBJECT *this.
			Linkage f_stepForwardTorques(const double dt, Eigen::VectorXd torques);

			//Uses the angles, angular accelerations, and positions as input.
			//Returns the cartsian positions, velocities, and accelerations for the current state.
			//[theta1 theta2 theta3 omega1 omega2 omega3 alpha1 alpha2 alpha3].
			//Eigen::VectorXd f_findCartesianMotionQuantities() const;	//TODO: UNFINISHED

			//TODO: Update angular accelerations function that uses dynamicsTorques().

			//Default constructor, primarily useful for testing so we fill with some arbitrary values.  Defaults to 3 link linkage.
			Linkage();
	};


}

