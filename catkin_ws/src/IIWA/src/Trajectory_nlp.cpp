// Copyright (C) 2009 International Business Machines.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: Trajectory_nlp.cpp 2005 2011-06-06 12:55:16Z stefan $
//
// Author:  Andreas Waechter               IBM    2009-04-02

// This file is part of the Ipopt tutorial.  It is a correct version
// of a C++ implemention of the coding exercise problem (in AMPL
// formulation):
//
// param n := 4;
//
// var x {1..n} <= 0, >= -1.5, := -0.5;
//
// minimize obj:
//   sum{i in 1..n} (x[i]-1)^2;
//
// subject to constr {i in 2..n-1}:
//   (x[i]^2+1.5*x[i]-i/n)*cos(x[i+1]) - x[i-1] = 0;
//
// The constant term "i/n" in the constraint is supposed to be input data
//
#define OPTIMIZE
#ifdef OPTIMIZE
#include "Trajectory_nlp.hpp"
#include "IpBlas.hpp"

#include <cstdio>
#include <cassert>
#include <cstdio>

// We use sin and cos
#include <cmath>

using namespace Ipopt;

// constructor
Trajectory_NLP::Trajectory_NLP(Index N, const Number g, Number v, 
	 const Number alph, const Number beta, const Number gam)
  :
  N_(N),  grav_(g), vel_(v), alph_(alph), beta_(beta), gam_(gam)
{
  //printf("constructing...");
  A_ = new double[9];
  B_ = new double[9];
  m_ = 4;
  x_l_ = new double[N];
  x_u_ = new double[N];
  // There are 4 constraints
  g_l_ = new double[m_];
  g_u_ = new double[m_];
    // constraints 0-2 are equalities to zero
  for (Index j=0; j<3; j++) {
    g_l_[j] = 0;
    g_u_[j] = 0;
  }
  // constraint 3 is equality to zero
  g_l_[3] = 1;
  g_u_[3] = 1;
  q_ = new double[3];
  dq_ = new double[3];
  pos_ = new double[3];
  bearing_ = new double[3];
  x0_ = new double[N];
  z_L0_ = new double[N];
  z_U0_ = new double[N];
  lambda0_ = new double[m_];

  x_sol_ = new double[N];
  z_l_sol_ = new double[N];
  z_u_sol_ = new double[N];
  lambda_sol_ = new double[m_];

  solved = false;
  //printf("constructed\n");

  
}

//destructor
Trajectory_NLP::~Trajectory_NLP()
{
  // make sure we delete everything we allocated
  delete [] x_l_;
  delete [] x_u_;
  delete [] g_l_;
  delete [] g_u_;
  delete [] q_;
  delete [] dq_;
  delete [] x0_;
  delete [] pos_;
  delete [] bearing_;
  delete [] x_sol_;
  delete [] z_l_sol_;
  delete [] z_u_sol_;
  delete [] lambda_sol_;
}

bool Trajectory_NLP::set_direction_weights(Number* A, Number* B)
{
  for (Index i=0; i<9; i++){
    A_[i] = A[i];
    B_[i] = B[i];
  }
  return true;
}

// returns the size of the problem
bool Trajectory_NLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
				   Index& nnz_h_lag,
				   IndexStyleEnum& index_style)
{
  //printf("getting nlp info...");
  // number of variables is given in constructor
  n = N_;

  // we have 4 constraints
  m = m_;

  // full jacobian n*m
  nnz_jac_g = n*m;

  // We have the full upper diagonals of 3x3 and 4x4 blocks;
  nnz_h_lag = 6 + 10;

  // use the C style indexing (0-based) for the matrices
  index_style = TNLP::C_STYLE;
  //printf("nlp info gotten\n");
  return true;
}

// sets the variable bounds
bool Trajectory_NLP::set_x_bounds_info(Number* x_l, Number* x_u)
{
  //printf("setting x bounds...");
  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  
  // the variables have lower bounds of -1.5
  for (Index i=0; i<N_; i++) {
    x_l_[i] = x_l[i];
    x_u_[i] = x_u[i];
  }
  //printf("x bounds set\n");

  solved = false;

  return true;
}

// returns the variable bounds
bool Trajectory_NLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
				      Index m, Number* g_l, Number* g_u)
{
  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  //printf("getting bounds info...");
  assert(n == N_);
  assert(m == m_);

   // the variables limits
  for (Index i=0; i<n; i++) {
    x_l[i] = x_l_[i];
    x_u[i] = x_u_[i];
  }

   // the constraints limits
  for (Index i=0; i<m; i++) {
    g_l[i] = g_l_[i];
    g_u[i] = g_u_[i];
  }
  //printf("bounds info gotten\n");
  return true;
}

bool Trajectory_NLP::set_q_info(Number* q, Number* dq)
{
  //printf("set q info");
  for(Index i=0; i<3; i++){
    q_[i] = q[i];
    dq_[i] = dq[i];
  }
  //printf("q info set\n");

  solved = false;

  return true;
}

// sets the initial point for the problem
bool Trajectory_NLP::set_starting_point(Number* robot_pos, Number* robot_bearing, bool init_x, Number* x,
					 bool init_z, Number* z_L, Number* z_U, bool init_lambda,
					 Number* lambda)
{

  // Here, we assume we only have starting values for x, if you code
  // your own NLP, you can provide starting values for the dual variables
  // if you wish
  //printf("setting start point...");
  init_x_ = init_x;
  init_z_ = init_z;
  init_lambda_ = init_lambda;

  for (Index i=0; i<3; i++){
    pos_[i] = robot_pos[i];
    bearing_[i] = robot_bearing[i];
  }

  // initialize to the given starting point
  if (init_x) {
    for (Index i=0; i<N_; i++) {
      x0_[i] = x[i];
    }
  }

  if (init_z) {
    for (Index i=0; i<N_; i++) {
      z_L0_[i] = z_L[i];
      z_U0_[i] = z_U[i];
    }
  }

  if (init_lambda) {
    for (Index i=0; i<m_; i++) {
      lambda0_[i] = lambda[i];
    }
  }

  solved = false;

  //printf("start point set\n");
  return true;
}
// returns the initial point for the problem
bool Trajectory_NLP::get_starting_point(Index n, bool init_x, Number* x,
					 bool init_z, Number* z_L, Number* z_U,
					 Index m, bool init_lambda,
					 Number* lambda)
{
  // Here, we assume we only have starting values for x, if you code
  // your own NLP, you can provide starting values for the dual variables
  // if you wish
  //printf("getting start point...");
  //printf("%d,%d\n", init_z ,init_z_);
  assert(init_x == init_x_);
  //assert(init_z == init_z_);
  //assert(init_lambda == init_lambda_);

  // initialize to the given starting point
  // initialize to the given starting point
  if (init_x) {
    for (Index i=0; i<n; i++) {
      x[i] = x0_[i];
    }
  }

  if (init_z) {
    for (Index i=0; i<n; i++) {
      z_L[i] = z_L0_[i];
      z_U[i] = z_U0_[i];
    }
  }

  if (init_lambda) {
    for (Index i=0; i<m; i++) {
      lambda[i] = lambda0_[i];
    }
  }
  //printf("start point gotten\n");
  return true;
}

// returns the value of the objective function
// f = a*(x-x0).'*A*(x-x0) + b*(xh-xh0).'*B*(xh-xh0) + c*tf
bool Trajectory_NLP::eval_f(Index n, const Number* x,
			     bool new_x, Number& obj_value)
{
  //printf("evaluating f...");
  //printf("evaluating objective\n");
  Number* dx = new double[3];
  Number* dx_hat = new double[3];
  // dx = x - x0,   dx_hat = x_hat - x_hat0
  for (Index i=0; i<3; i++){
    dx[i] = x[i] - pos_[i];
    dx_hat[i] = x[i+3] - bearing_[i];
  }
  
  Number* result = new double[3];
  // result = alph*A*dx
  IpBlasDgemv(true, 3, 3, alph_, A_, 3, dx, 1, 0, result, 1);
  // cost = dx' * result = alph * dx' * A * dx
  obj_value = IpBlasDdot(3, dx, 1, result,1);
  // result = beta*B*dx_hat
  IpBlasDgemv(true, 3, 3, beta_, B_, 3, dx_hat, 1, 0, result, 1);
  // cost = cost + dx_hat'*result + gam*t_flight
  // => cost = alph*dx'*A*dx + beta*dx_hat'*B*dx_hat + gam*t_flight //
  obj_value += IpBlasDdot(3, dx_hat, 1, result,1) + gam_*x[6];
  //printf("f evaluated\n");
  return true;
}

// return the gradient of the objective function grad_{x} f(x)
// gradf = [2*a*A*(x-x0); 2*b*B*(xh-xh0); gam]
bool Trajectory_NLP::eval_grad_f(Index n, const Number* x,
				  bool new_x, Number* grad_f)
{
  //printf("evaluating df...");
  Number* dx = new double[3];
  Number* dx_hat = new double[3];
  // dx = x - x0,   dx_hat = x_hat - x_hat0
  for (Index i=0; i<3; i++){
    dx[i] = x[i] - pos_[i];
    dx_hat[i] = x[i+3] - bearing_[i];
  }

  Number* result = new double[3];
  // result = 2*alph*A*dx
  IpBlasDgemv(true, 3, 3, 2*alph_, A_, 3, dx, 1, 0, result, 1);
  for(Index i=0; i<3; i++) {grad_f[i] = result[i];}
  
  // result = 2*beta*B*dx_hat
  IpBlasDgemv(true, 3, 3, 2*beta_, B_, 3, dx_hat, 1, 0, result, 1);
  for(Index i=0; i<3; i++) {grad_f[i+3] = result[i];}

  // grad_f[end] = gam
  grad_f[6] = gam_;
  //printf("grad_f: %f,%f,%f,%f,%f,%f,%f\n", grad_f[0],grad_f[1],grad_f[2],grad_f[3],grad_f[4],grad_f[5],grad_f[6]);
  //printf("df evaluated\n");
  return true;
}

// return the value of the constraints: g(x)
// ceq = [g*tf^2/2 + v*xh*tf + x - (dq*tf + q); xh.'*xh]
bool Trajectory_NLP::eval_g(Index n, const Number* x,
			     bool new_x, Index m, Number* g)
{
  //printf("evaluating g...");
  for (Index i=0; i<2; i++) {
  	g[i] = (vel_*x[i+3] - dq_[i])*x[6] + x[i] - q_[i];
  }
  // x[2] direction has gravity
  g[2] = 0.5*grav_*x[6]*x[6] + (vel_*x[2+3] - dq_[2])*x[6] + x[2] - q_[2];
  g[3] = x[3]*x[3] + x[4]*x[4] + x[5]*x[5];
  //printf("g evaluated\n");
  return true;
}

// return the structure or values of the jacobian
/* 
    gradceq = [ 1, 0, 0, 0;
         0, 1, 0, 0;
         0, 0, 1, 0;
         tf*v*eye(3), 2*xh;
         (g*tf + (v*xh-dq)).', 0]'  (note the transpose)
 */
bool Trajectory_NLP::eval_jac_g(Index n, const Number* x, bool new_x,
				 Index m, Index nele_jac, Index* iRow,
				 Index *jCol, Number* values)
{
  //printf("evaluating dg...");
  if (values == NULL) {
    // return the structure of the jacobian

    Index inz = 0;
    for (Index j=0; j<m; j++) {
      for(Index i=0; i<n; i++){
        iRow[inz] = j;
        jCol[inz] = i;
        inz++;
      }
    }
    // sanity check
    assert(inz==nele_jac);
  }
  else {
    // return the values of the jacobian of the constraints

    values[0] = 1;  values[1] = 0;  values[2] = 0;  values[3] = x[6]*vel_; values[4] = 0;          values[5] = 0;          values[6] = vel_*x[3] - dq_[0];
    values[7] = 0;  values[8] = 1;  values[9] = 0;  values[10] = 0;        values[11] = x[6]*vel_; values[12] = 0;         values[13] = vel_*x[4] - dq_[1]; 
    values[14] = 0; values[15] = 0; values[16] = 1; values[17] = 0;        values[18] = 0;         values[19] = x[6]*vel_; values[20] = grav_*x[6] + vel_*x[5] - dq_[2];
    values[21] = 0; values[22] = 0; values[23] = 0; values[24] = 2*x[3];   values[25] = 2*x[4];    values[26] = 2*x[5];    values[27] = 0;
    //printf("jac_g:\n%f,%f,%f,%f,%f,%f,%f\n%f,%f,%f,%f,%f,%f,%f\n%f,%f,%f,%f,%f,%f,%f\n%f,%f,%f,%f,%f,%f,%f\n",
  //	values[0],values[1],values[2],values[3],values[4],values[5],values[6],
  //	values[7],values[8],values[9],values[10],values[11],values[12],values[13],
  //	values[14],values[15],values[16],values[17],values[18],values[19],values[20],
  //	values[21],values[22],values[23],values[24],values[25],values[26],values[27]);
  }

  //printf("dg evaluated\n");
  return true;
}

//return the structure or values of the hessian
bool Trajectory_NLP::eval_h(Index n, const Number* x, bool new_x,
			     Number obj_factor, Index m, const Number* lambda,
			     bool new_lambda, Index nele_hess, Index* iRow,
			     Index* jCol, Number* values)
{
  //printf("evaluating h...");
  if (values == NULL) {

    Index inz = 0;
    // first elements are upper triangle of upper 3x3 block
    for (Index i=0; i<3; i++) {
      for(Index j=i; j<3; j++) {
        iRow[inz] = i;
        jCol[inz] = j;
        inz++;
      }
    }

    // define next elements as upper triangle of lower 4x4 block
    for (Index i=3; i<n; i++) {
      for(Index j=i; j<n; j++) {
        iRow[inz] = i;
        jCol[inz] = j;
        inz++;
      }
    }
    //printf("ASSERT %d,%d\n",inz,nele_hess);
    assert(inz == nele_hess);
  }
  else {
    // return the values. This is a symmetric matrix, fill the upper right
    // triangle only

    Index inz = 0;
    // first elements are upper triangle of upper 3x3 block
    for (Index i=0; i<3; i++) {
      for(Index j=i; j<3; j++) {
        values[inz] = obj_factor*A_[j+i*3];
        inz++;
      }
    }
    
    // define next elements as upper triangle of lower 4x4 block
    values[inz] = obj_factor*B_[0] + lambda[3]*2;
    inz++;
    values[inz] = obj_factor*B_[1];
    inz++;
    values[inz] = obj_factor*B_[2];
    inz++;
    values[inz] = lambda[0]*vel_;
    inz++;
    // next row
    values[inz] = obj_factor*B_[4] + lambda[3]*2;
    inz++;
    values[inz] = obj_factor*B_[5];
    inz++;
    values[inz] = lambda[1]*vel_;
    inz++;
    // next row
    values[inz] = obj_factor*B_[8] + lambda[3]*2;
    inz++;
    values[inz] = lambda[2]*vel_;
    inz++;
    // next row (last)
    values[inz] = lambda[2]*grav_;
    inz++;
    // Diagonal entry for first variable
    //values[inz] = obj_factor*2.;
    //inz++;
//printf("ASSERT %d,%d\n",inz,nele_hess);

    // sanity check
    assert(inz == nele_hess);
    //printf("\n %f,%f,%f\n\t  %f,%f\n\t\t   %f\n%f,%f,%f,%f\n\t %f,%f,%f\n\t\t %f,%f\n\t\t\t   %f\n",
  //	values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7],values[8],
  //	values[9],values[10],values[11],values[12],values[13],values[14],values[15]);
  }
  //printf("h evaluated\n");
  return true;
}

void Trajectory_NLP::finalize_solution(SolverReturn status,
					Index n, const Number* x,
					const Number* z_L, const Number* z_U,
					Index m, const Number* g,
					const Number* lambda,
					Number obj_value,
					const IpoptData* ip_data,
					IpoptCalculatedQuantities* ip_cq)
{
  // here is where we would store the solution to variables, or write
  // to a file, etc so we could use the solution.

  for (Index i=0; i<N_; i++){
    x_sol_[i] = x[i];
    z_l_sol_[i] = z_L[i];
    z_u_sol_[i] = z_U[i];
  }


  for (Index i=0; i<m_; i++){
    lambda_sol_[i] = lambda[i];
  }

  solved = true;
/*
  //printf("\nWriting solution file solution.txt\n");
  FILE* fp = fopen("solution.txt", "w");

  // For this example, we write the solution to the console
  fprintf(fp, "\n\nSolution of the primal variables, x\n");
  for (Index i=0; i<n; i++) {
    fprintf(fp, "x[%d] = %e\n", i, x[i]);
    printf("x[%d] = %e\n", i, x[i]);
  }

  fprintf(fp, "\n\nSolution of the bound multipliers, z_L and z_U\n");
  for (Index i=0; i<n; i++) {
    fprintf(fp, "z_L[%d] = %e\n", i, z_L[i]);
  }
  for (Index i=0; i<n; i++) {
    fprintf(fp, "z_U[%d] = %e\n", i, z_U[i]);
  }

  fprintf(fp, "\n\nObjective value\n");
  fprintf(fp, "f(x*) = %e\n", obj_value);
  fclose(fp);
*/
}

bool Trajectory_NLP::get_solution(Number* x, Number* z_L, Number* z_U, Number* lambda)
{
  if (solved){
    for (Index i=0; i<N_; i++){
      x[i] = x_sol_[i];
      z_L[i] = z_l_sol_[i];
      z_U[i] = z_u_sol_[i];
    }

    for (Index i=0; i<m_; i++){
      lambda[i] = lambda_sol_[i];
    }
    return true;
  }
  return false;
}

void Trajectory_NLP::print_params(){
  printf("N_: %d\nm_: %d\ngrav_: %f\nvel_: %f\nA_: %f,%f,%f\n    %f,%f,%f\n    %f,%f,%f\nB_: %f,%f,%f\n    %f,%f,%f\n    %f,%f,%f\nalph_: %f\nbeta_: %f\ngam_: %f\nx_l_: %f,%f,%f,%f,%f,%f,%f\nx_u_: %f,%f,%f,%f,%f,%f,%f\ng_l_: %f,%f,%f,%f\ng_u_: %f,%f,%f,%f\nq_: %f,%f,%f\ndq_: %f,%f,%f\nx0_: %f,%f,%f,%f,%f,%f,%f\nz_L0_: %f\nz_U0_: %f\nlambda0_: %f,%f,%f,%f\n",
  	N_,m_,grav_,vel_,
  	A_[0],A_[1],A_[2],A_[3],A_[4],A_[5],A_[6],A_[7],A_[8],
  	B_[0],B_[1],B_[2],B_[3],B_[4],B_[5],B_[6],B_[7],B_[8],
  	alph_, beta_, gam_,
  	x_l_[0],x_l_[1],x_l_[2],x_l_[3],x_l_[4],x_l_[5],x_l_[6],
  	x_u_[0],x_u_[1],x_u_[2],x_u_[3],x_u_[4],x_u_[5],x_u_[6],
  	g_l_[0],g_l_[1],g_l_[2],g_l_[3],
  	g_u_[0],g_u_[1],g_u_[2],g_u_[3],
  	q_[0],q_[1],q_[2],
  	dq_[0],dq_[1],dq_[2],
  	x0_[0],x0_[1],x0_[2],x0_[3],x0_[4],x0_[5],x0_[6],
  	0.0, 0.0,
  	0.0,0.0,0.0,0.0);//lambda0_[0],lambda0_[1],lambda0_[2],lambda0_[3]
}
#endif
