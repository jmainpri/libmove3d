/*
 *  RoboptimTrajectory.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 22/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "RoboptimTrajectory.h"

#include <roboptim/trajectory/sys.hh>

#include <fstream>

#include <boost/assign/list_of.hpp>
#include <boost/mpl/vector.hpp>

#include <roboptim/core/io.hh>
#include <roboptim/core/finite-difference-gradient.hh>
//#include <roboptim/core/solver-factory.hh>
#include <roboptim/core/solver-factory.hh>

#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-commands.hh>
#include <roboptim/core/visualization/gnuplot-function.hh>

#include <roboptim/trajectory/free-time-trajectory.hh>
#include <roboptim/trajectory/freeze.hh>
#include <roboptim/trajectory/fwd.hh>
#include <roboptim/trajectory/limit-speed.hh>
#include <roboptim/trajectory/spline-length.hh>
#include <roboptim/trajectory/cubic-b-spline.hh>
#include <roboptim/trajectory/trajectory-cost.hh>

#include <roboptim/trajectory/visualization/limit-speed.hh>


#include <roboptim/core/plugin/cfsqp.hh>


//#include "common.hh"

using namespace roboptim;
using namespace roboptim::visualization;
using namespace roboptim::visualization::gnuplot;

typedef CFSQPSolver::problem_t::constraints_t constraint_t;
typedef CFSQPSolver solver_t;
typedef FreeTimeTrajectory<CubicBSpline> freeTime_t;


// Problem parameters.
const unsigned nControlPoints = 15;
const unsigned nConstraintsPerCtrlPts = 10;
const double vMax = 85.;

/*
int run_test ()
{
	using namespace boost;
	using namespace boost::assign;
	
	const double finalPos = 200.;
	CubicBSpline::vector_t params (nControlPoints);
	
	params[0] = 0;
	params[1] = 0;
	
	for (unsigned i = 0; i < nControlPoints-4; ++i)
		params[i+2] = finalPos / (nControlPoints - 5) * i;
	
	params[nControlPoints-2] = finalPos;
	params[nControlPoints-1] = finalPos;
	
	// Make trajectories.
	CubicBSpline::interval_t timeRange = CubicBSpline::makeInterval (0., 4.);
	CubicBSpline spline (timeRange, 1, params, "before");
	freeTime_t freeTimeTraj (spline, 1.);
	
	// Define cost.
	Function::matrix_t a (1, freeTimeTraj.parameters ().size ());
	a.clear ();
	a (0, 0) = -1.;
	Function::vector_t b (1);
	b.clear ();
	roboptim::NumericLinearFunction cost (a, b);
	
	// Create problem.
	solver_t::problem_t problem (cost);
	problem.startingPoint () = freeTimeTraj.parameters ();
	
	// Scale has to remain positive.
	problem.argumentBounds ()[0] = Function::makeLowerInterval (0.);
	
	const freeTime_t::vector_t freeTimeParams = freeTimeTraj.parameters ();
	
	std::vector<Function::size_type> indices;
	indices.push_back (1);
	indices.push_back (2);
	indices.push_back (3);
	indices.push_back (freeTimeParams.size () - 3);
	indices.push_back (freeTimeParams.size () - 2);
	indices.push_back (freeTimeParams.size () - 1);
	makeFreeze (problem) (indices, freeTimeParams);
	
	Function::interval_t vRange = Function::makeUpperInterval (.5 * vMax * vMax);
	LimitSpeed<FreeTimeTrajectory<CubicBSpline> >::addToProblem
	(freeTimeTraj, problem, vRange, nControlPoints * nConstraintsPerCtrlPts);
	
	std::ofstream limitSpeedStream ("limit-speed.gp");
	Gnuplot gnuplot = Gnuplot::make_interactive_gnuplot ();
	
	gnuplot
	<< set ("multiplot layout 1,2 title "
			"'variation of speed before and after optimization'")
	<< set ("grid");
	gnuplot << plot_limitSpeed (freeTimeTraj, vMax);
	
	
	// SolverFactory<solver_t> factory ("cfsqp", problem);
	// solver_t& solver = factory ();
	
	// Initialize solver
	CFSQPSolver solver (problem);
	
	std::cout << solver << std::endl;
	
	solver_t::result_t res = solver.minimum ();
	std::cerr << res << std::endl;
	
	FreeTimeTrajectory<CubicBSpline> optimizedTrajectory =
	freeTimeTraj;
	
	switch (solver.minimumType ())
	{
		case GenericSolver::SOLVER_VALUE:
		{
			const Result& result = solver.getMinimum<Result> ();
			optimizedTrajectory.setParameters (result.x);
			break;
		}
			
		case GenericSolver::SOLVER_VALUE_WARNINGS:
		{
			const ResultWithWarnings& result =
			solver.getMinimum<ResultWithWarnings> ();
			optimizedTrajectory.setParameters (result.x);
			break;
		}
			
		case GenericSolver::SOLVER_NO_SOLUTION:
		case GenericSolver::SOLVER_ERROR:
			return 1;
	}
	
	gnuplot << plot_limitSpeed (optimizedTrajectory, vMax);
	limitSpeedStream << (gnuplot << unset ("multiplot"));
	return 0;
}
 */
/*
 struct F : public TwiceDerivableFunction
 {
 F () : TwiceDerivableFunction (4, 1)
 {
 }
 
 void
 impl_compute (result_t& result, const argument_t& x) const throw ()
 {
 vector_t res (m);
 res (0) = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[3];
 return res;
 }
 
 void
 impl_gradient (gradient_t& grad, const argument_t& x, int) const throw ()
 {
 gradient_t grad (n);
 
 grad[0] = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]);
 grad[1] = x[0] * x[3];
 grad[2] = x[0] * x[3] + 1;
 grad[3] = x[0] * (x[0] + x[1] + x[2]);
 return grad;
 }
 
 void
 impl_hessian (hessian_t& h, const argument_t& x, int) const throw ()
 
 {
 matrix_t h (n, n);
 h (0, 0) = 2 * x[3];
 h (0, 1) = x[3];
 h (0, 2) = x[3];
 h (0, 3) = 2 * x[0] + x[1] + x[2];
 
 h (1, 0) = x[3];
 h (1, 1) = 0.;
 h (1, 2) = 0.;
 h (1, 3) = x[0];
 
 h (2, 0) = x[3];
 h (2, 1) = 0.;
 h (2, 2) = 0.;
 h (2, 3) = x[1];
 
 h (3, 0) = 2 * x[0] + x[1] + x[2];
 h (3, 1) = x[0];
 h (3, 2) = x[0];
 h (3, 3) = 0.;
 return h;
 }
 };
 
 struct G0 : public TwiceDerivableFunction
 {
 G0 ()
 : TwiceDerivableFunction (4, 1)
 {
 }
 
 void
 impl_compute (result_t& result, const argument_t& x) const throw ()
 {
 vector_t res (m);
 res (0) = x[0] * x[1] * x[2] * x[3];
 return res;
 }
 
 void
 impl_gradient (gradient_t& grad, const argument_t& x, int) const throw ()
 {
 gradient_t grad (n);
 
 grad[0] = x[1] * x[2] * x[3];
 grad[1] = x[0] * x[2] * x[3];
 grad[2] = x[0] * x[1] * x[3];
 grad[3] = x[0] * x[1] * x[2];
 return grad;
 }
 
 void
 impl_hessian (hessian_t& h, const argument_t& x, int) const throw ()
 {
 matrix_t h (n, n);
 h (0, 0) = 0.;
 h (0, 1) = x[2] * x[3];
 h (0, 2) = x[1] * x[3];
 h (0, 3) = x[1] * x[2];
 
 h (1, 0) = x[2] * x[3];
 h (1, 1) = 0.;
 h (1, 2) = x[0] * x[3];
 h (1, 3) = x[0] * x[2];
 
 h (2, 0) = x[1] * x[3];
 h (2, 1) = x[0] * x[3];
 h (2, 2) = 0.;
 h (2, 3) = x[0] * x[1];
 
 h (3, 0) = x[1] * x[2];
 h (3, 1) = x[0] * x[2];
 h (3, 2) = x[0] * x[1];
 h (3, 3) = 0.;
 return h;
 }
 };
 
 struct G1 : public TwiceDerivableFunction
 {
 G1 ()
 : TwiceDerivableFunction (4, 1)
 {
 }
 
 void
 impl_compute (result_t& result, const argument_t& x) const throw ()
 {
 vector_t res (m);
 res (0) = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];
 return res;
 }
 
 void
 impl_gradient (gradient_t& grad, const argument_t& x, int) const throw ()
 {
 gradient_t grad (n);
 
 grad[0] = 2 * x[0];
 grad[1] = 2 * x[1];
 grad[2] = 2 * x[2];
 grad[3] = 2 * x[3];
 return grad;
 }
 
 void
 impl_hessian (hessian_t& h, const argument_t& x, int) const throw ()
 {
 matrix_t h (n, n);
 h (0, 0) = 2.;
 h (0, 1) = 0.;
 h (0, 2) = 0.;
 h (0, 3) = 0.;
 
 h (1, 0) = 0.;
 h (1, 1) = 2.;
 h (1, 2) = 0.;
 h (1, 3) = 0.;
 
 h (2, 0) = 0.;
 h (2, 1) = 0.;
 h (2, 2) = 2.;
 h (2, 3) = 0.;
 
 h (3, 0) = 0.;
 h (3, 1) = 0.;
 h (3, 2) = 0.;
 h (3, 3) = 2.;
 return h;
 }
 };
 
 void run_test() {
 F f;z
 G0 g0;
 G1 g1;
 
 CFSQPSolver::problem_t pb (f);
 
 // Set bound for all variables.
 // 1. < x_i < 5. (x_i in [1.;5.])
 for (Function::size_type i = 0; i < pb.function ().n; ++i)
 pb.argBounds ()[i] = T::makeBound (1., 5.);
 
 // Add constraints.
 pb.addConstraint (&g0, T::makeUpperBound (25.));
 pb.addConstraint (&g1, T::makeBound (40., 40.));
 
 // Set the starting point.
 Function::vector_t start (pb.function ().n);
 start[0] = 1., start[1] = 5., start[2] = 5., start[3] = 1.;
 
 initialize_problem (pb, g0, g1);
 
 // Initialize solver
 CFSQPSolver solver (pb);
 
 // Compute the minimum and retrieve the result.
 CFSQPSolver::result_t res = solver.minimum ();
 
 // Display solver information.
 std::cout << solver << std::endl;
 
 // Check if the minimization has succeed.
 switch (solver.minimumType ())
 {
 case SOLVER_NO_SOLUTION:
 std::cerr << "No solution." << std::endl;
 return 1;
 case SOLVER_ERROR:
 std::cerr << "An error happened: "
 << solver.getMinimum<SolverError> ().what () << std::endl;
 return 2;
 
 case SOLVER_VALUE_WARNINGS:
 {
 // Get the ``real'' result.
 Result& result = solver.getMinimum<ResultWithWarnings> ();
 // Display the result.
 std::cout << "A solution has been found (minor problems occurred): "
 << std::endl
 << result << std::endl;
 return 0;
 }
 case SOLVER_VALUE:
 {
 // Get the ``real'' result.
 Result& result = solver.getMinimum<Result> ();
 // Display the result.
 std::cout << "A solution has been found: " << std::endl;
 std::cout << result << std::endl;
 return 0;
 }
 }
 
 // Should never happen.
 assert (0);
 return 42;
 }
 //GENERATE_TEST ()*/