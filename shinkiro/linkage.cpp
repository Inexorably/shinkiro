#include "linkage.h"

#define SHOW_DEBUG_OUTPUT true

/**********************************************************Base functions (constructor etc)************************************************/

//Default Link constructor, primarily useful for testing so we fill with some arbitrary values.
shinkiro::Link::Link() : m_length(2), m_radius(1), m_mass(1), m_inertiaCenter(1), m_theta(EIGEN_PI/4), m_omega(1), m_alpha(1), m_forceX(1), m_forceY(-1), m_torque(1) {
	if (SHOW_DEBUG_OUTPUT) {
		std::cout << "shinkiro::Link::Link(): Link constructed succesfully.\n";
	}
}

//Default Linkage constructor, primarily useful for testing so we fill with some arbitrary values.  Defaults to 3 link linkage.
shinkiro::Linkage::Linkage() {
	//We default to a 3 link linkage with the default link characteristics generated by the default constructor of Link.
	shinkiro::Link temp;

	//Fill the Link vector.
	for (int i = 0; i < 3; ++i) {
		m_links.push_back(temp);
	}
	if (SHOW_DEBUG_OUTPUT) {
		std::cout << "shinkiro::Linkage::Linkage(): Linkage constructed succesfully.\n";
	}
}

/**********************************************************Math helper functions***********************************************************/

//Returns cos(theta)*alpha.
double shinkiro::Link::f_cta() const {
	return cos(m_theta) * m_alpha;
}

//Returns sin(theta)*alpha.
double shinkiro::Link::f_sta() const {
	return sin(m_theta) * m_alpha;
}

//Returns cos(theta)*omega^2.
double shinkiro::Link::f_ctw2() const {
	return cos(m_theta) * pow(m_omega, 2);
}

//Returns sin(theta)*omega^2.
double shinkiro::Link::f_stw2() const {
	return sin(m_theta) * pow(m_omega, 2);
}

/**********************************************************Solver functions***************************************************************/

//Use inverse dynamics to calculate the forces and moments for a three link planar bipedal / inverted pendulum model.
//Assumes foot is massless and remains grounded for period of analysis.  Assumes that position, velocity, and acceleration values are known and defined in m_theta, m_omega, m_alpha.
//Returns a vector of the forces and torques of order [Fx1; Fx2; Fy1; Fy2; T1; T2; Fx3; Fy3; T3].
Eigen::VectorXd shinkiro::Linkage::f_inverseDynamics() const {
	//Handle the cases for the current number of links.  Eventually generically handle linkages, but for now prototype around 3 link model.
	if (m_links.size() != 3) {
		std::cout << "Invalid linkage length\n";
		return Eigen::VectorXd(8);
	}

	//The inverse dynamics for linkage of length three can be written in the form Af=b.
	//Prellocate A and b.
	Eigen::MatrixXd A(9, 9);
	Eigen::VectorXd b(9);

	//Define the matrix A, which is 9x9.
	A << 1, -1, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, -1, 0, 0, 0, 0, 0,
		1, -1, m_links[0].m_radius* sin(m_links[0].m_theta), -1 * m_links[0].m_radius * cos(m_links[0].m_theta), (m_links[0].m_length - m_links[0].m_radius)* sin(m_links[0].m_theta), (m_links[0].m_length - m_links[0].m_radius)* cos(m_links[0].m_theta), 0, 0, 0,
		0, 1, 0, 0, 0, 0, -1, 0, 0,
		0, 0, 0, 1, 0, 0, 0, -1, 0,
		0, m_links[1].m_radius* sin(m_links[1].m_theta), 0, -1 * m_links[1].m_radius * cos(m_links[1].m_theta), 0, 1, (m_links[1].m_length - m_links[1].m_radius)* sin(m_links[1].m_theta), -1 * (m_links[1].m_length - m_links[1].m_radius) * cos(m_links[1].m_theta), -1,
		0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, m_links[2].m_radius* sin(m_links[2].m_theta), -m_links[2].m_radius * cos(m_links[2].m_theta), 1;

	//Define the vector B, which is 9x1.
	b << -m_links[0].m_mass * m_links[0].m_radius * (m_links[0].f_sta() + m_links[0].f_ctw2()),
		m_links[0].m_mass* (m_links[0].m_radius * (m_links[0].f_cta() - m_links[0].f_stw2()) + shinkiro::g),
		m_links[0].m_inertiaCenter* m_links[0].m_alpha,
		-m_links[1].m_mass * (m_links[0].m_length * (m_links[0].f_sta() + m_links[0].f_ctw2()) + m_links[1].m_radius * (m_links[1].f_sta() + m_links[1].f_ctw2())),
		m_links[1].m_mass* (m_links[0].m_length * (m_links[0].f_cta() - m_links[0].f_stw2()) + m_links[1].m_radius * (m_links[1].f_cta() - m_links[1].f_stw2()) + shinkiro::g),
		m_links[1].m_inertiaCenter* m_links[1].m_alpha,
		-m_links[2].m_mass * (m_links[0].m_length * (m_links[0].f_sta() + m_links[0].f_ctw2()) + m_links[1].m_length * (m_links[1].f_sta() + m_links[1].f_ctw2()) + m_links[2].m_radius * (m_links[2].f_sta() + m_links[2].f_ctw2())),
		m_links[2].m_mass* (m_links[0].m_length * (m_links[0].f_cta() - m_links[0].f_stw2()) + m_links[1].m_length * (m_links[1].f_cta() - m_links[1].f_stw2()) + m_links[2].m_radius * (m_links[2].f_cta() - m_links[2].f_stw2()) + shinkiro::g),
		m_links[2].m_inertiaCenter* m_links[2].m_alpha;


	//Find f = b\A.
	return A.colPivHouseholderQr().solve(b);
}

//Use forward dynamics to find the angular accelerations resulting from some given forces and torques.
//Assumes foot is massless and remains grounded for period of analysis.
//Returns a vector of angular accelerations of order [alpha1; alpha2; alpha3].
Eigen::VectorXd shinkiro::Linkage::f_forwardDynamicsFull() const {
	//Handle the cases for the current number of links.  Eventually generically handle linkages, but for now prototype around 3 link model.
	if (m_links.size() != 3) {
		std::cout << "Invalid linkage length\n";
		return Eigen::VectorXd(3);
	}
	
	//Find the angular accelerations from the moment balance equations.
	//TODO: Adapt this function for Linkage with n links, as the moment equations are consistently the same.
	double alpha1 = (m_links[0].m_torque - m_links[1].m_torque + m_links[0].m_forceX * m_links[0].m_radius * sin(m_links[0].m_theta) - m_links[0].m_forceY * m_links[0].m_radius * cos(m_links[0].m_theta)
		+ m_links[1].m_forceX * (m_links[0].m_length - m_links[0].m_radius) * sin(m_links[0].m_theta) - m_links[1].m_forceY * (m_links[0].m_length - m_links[0].m_radius) * cos(m_links[0].m_theta))
		/ m_links[0].m_inertiaCenter;

	double alpha2 = (m_links[1].m_torque - m_links[2].m_torque + m_links[1].m_forceX * m_links[1].m_radius * sin(m_links[1].m_theta) - m_links[1].m_forceY * m_links[1].m_radius * cos(m_links[1].m_theta)
		+ m_links[2].m_forceX * (m_links[1].m_length - m_links[1].m_radius) * sin(m_links[1].m_theta) - m_links[2].m_forceY * (m_links[1].m_length - m_links[1].m_radius) * cos(m_links[1].m_theta))
		/ m_links[1].m_inertiaCenter;

	//For the final link, the end forces are zero.
	double alpha3 = (m_links[2].m_torque + m_links[2].m_forceX * m_links[2].m_radius * sin(m_links[2].m_theta) - m_links[2].m_forceY * m_links[2].m_radius * cos(m_links[2].m_theta))
		/ m_links[2].m_inertiaCenter;

	/*
	//Check alpha1 from the equation 1.
	double alpha1a = ((m_links[0].m_forceX - m_links[1].m_forceX) / (-m_links[0].m_mass * m_links[0].m_radius) - cos(m_links[0].f_ctw2())) / sin(m_links[0].m_theta);
	*/

	//Store and return the angular accelerations.
	Eigen::VectorXd temp(m_links.size());
	temp << alpha1, alpha2, alpha3;

	return temp;
}

//Use forward dynamics to find the angular accelerations resulting from torques only.
//Assumes foot is massless and remains grounded for period of analysis.
//Returns a vector of angular accelerations of order [alpha1; alpha2; alpha3].
Eigen::VectorXd shinkiro::Linkage::f_forwardDynamicsTorques(Eigen::VectorXd torques) const {
	//Handle the cases for the current number of links.  Eventually generically handle linkages, but for now prototype around 3 link model.
	if (m_links.size() != 3) {
		std::cout << "Invalid linkage length\n";
		return Eigen::VectorXd(3);
	}

	if (torques.size() != 3) {
		std::cout << "Invalid torque vector length\n";
		return Eigen::VectorXd(3);
	}

	//Find the angular accelerations from the moment balance equations, moment around bottom pin joint (link origin).
	//TODO: Account for reaction moments from masses of other links
	double alpha1 = (torques[0] - torques[1] - m_links[0].m_mass * shinkiro::g * m_links[0].m_radius) / m_links[0].m_inertiaOrigin;
	double alpha2 = (torques[1] - torques[2] - m_links[1].m_mass * shinkiro::g * m_links[1].m_radius) / m_links[1].m_inertiaOrigin;
	double alpha3 = (torques[2] - m_links[3].m_mass * shinkiro::g * m_links[3].m_radius) / m_links[2].m_inertiaOrigin;

	//Store and return the angular accelerations.
	Eigen::VectorXd temp(m_links.size());
	temp << alpha1, alpha2, alpha3;

	return temp;
}

//Returns a linkage stepped forward using some input torques.
//Returns a copy of the result.  NOTE: INTEGRATES CURRENT LINKAGE OBJECT *this.
shinkiro::Linkage shinkiro::Linkage::f_stepForwardTorques(const double dt, Eigen::VectorXd torques) {
	//Get the angular accelerations for the given torques.
	Eigen::VectorXd alphas = f_forwardDynamicsTorques(torques);

	//Update the new angular velocities and positions.
	for (int i = 0; i < m_links.size(); ++i) {
		m_links[i].m_omega = m_links[i].m_omega + alphas[i] * dt;
		m_links[i].m_theta = m_links[i].m_theta + m_links[i].m_omega * dt;
	}
	
	//Return a copy if desired for plotting purposes.
	return *this;
}

//Uses the angles, angular accelerations, and positions as input.
//Returns the cartsian positions, velocities, and accelerations for the current state.
/*
Eigen::VectorXd shinkiro::Linkage::f_findCartesianMotionQuantities() const {
	//Handle the cases for the current number of links.  Eventually generically handle linkages, but for now prototype around 3 link model.
	if (m_links.size() != 3) {
		std::cout << "Invalid linkage length\n";
		return Eigen::VectorXd(9);
	}

	//Initialize variables.
	double px1, px2, px3, py1, py2, py3, vx1, vx2, vx3, vy1, vy2, vy3, ax1, ax2, ax3, ay1, ay2, ay3;

	//Link 1.
	px1 = m_links[0].m_radius * cos(m_links[0].m_theta);
	py1 = m_links[0].m_radius * sin(m_links[0].m_theta);
	vx1 = -m_links[0].m_radius * sin(m_links[0].m_theta) * m_links[0].m_omega;
	vy1 = m_links[0].m_radius * cos(m_links[0].m_theta) * m_links[0].m_omega;

	//Store and return the angular accelerations.
	Eigen::VectorXd temp(m_links.size());

	std::cout << "warning f_findCartesianMotionQuantities()\n";

	return temp;
}
*/