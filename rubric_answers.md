# Rubric Answers

By Steven Eisinger

## The Model

The model being used here is the bicycle model seen from the Vehicle Models lesson. The following steps are taken:

1. The current state is retrieved from the simulator
1. The retrieved coordinates are transformed such that the car is placed at the origin
1. The state of the car in 100ms is predicted to generate the state
1. The state is passed to the MPC solver
1. The solver predicts the path with 10 actuations taking into accounts specific weights and returns them in the vars vector
1. The first actuation in the solver's prediction is performed
1. The process repeats from step 1

The cost functions for actuations can be seen in the `FG_eval` funtion:

```c++
// The part of the cost based on the reference state.
for (size_t t = 0; t < N; t++) {
    fg[0] += 10 * CppAD::pow(vars[cte_start + t], 2);
    fg[0] += 10 * CppAD::pow(vars[epsi_start + t], 2);
    fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
}

// Minimize the use of actuators.
for (size_t t = 0; t < N - 1; t++) {
    fg[0] += 150 * CppAD::pow(vars[delta_start + t], 2);
    fg[0] += 150 * CppAD::pow(vars[a_start + t], 2);
}

// Minimize the value gap between sequential actuations.
for (size_t t = 0; t < N - 2; t++) {
    fg[0] += 5000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
    fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
```

The reference velocity `ref_v` was chosen to be 100 mph so that the car attempts to go quite fast. In reality, the weights above result in the car going at about 50 mph.

Equations corresponding to the bicycle model can be seen in `MPC.cpp` in the following lines within `FG_eval`:

```c++
// Model equations
fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[2 + psi_start + i] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
```

Constraints for actuation are also found in `MPC.cpp`. The constraints for the steeringt angle are chosen to be 25 degrees as converted to radions, multiplied by LF so that when we divide by Lf later, we get exaclty 25 degrees as our boundary. The throttle is limmited between -1 and 1 (full break to full throttle). The limits of the other variables are just set to very large numbers. They can be seen below:

```c++
Dvector vars_lowerbound(n_vars);
Dvector vars_upperbound(n_vars);
// Set lower and upper limits for variables.
for (i = 0; i < delta_start; i++) {
vars_lowerbound[i] = -1.0e19;
vars_upperbound[i] = 1.0e19;
}

// Degrees value in radians
for (i = delta_start; i < a_start; i++) {
vars_lowerbound[i] = -0.436332*Lf;
vars_upperbound[i] = 0.436332*Lf;
}

// Acceleration/deceleration limits
for (i = a_start; i < n_vars; i++) {
vars_lowerbound[i] = -1.0;
vars_upperbound[i] = 1.0;
}
```

The `Ipopt` module is responsible for finding the polynomial fit based on actuator constraints and the path to follow. The `mpc.Solve` function outputs the vector `vars` in `main.cpp` which are use to update the actuators as seen here:

```c++
msgJson["steering_angle"] = vars[0] / (deg2rad(25) * Lf);
msgJson["throttle"] = vars[1];`
```

## Timestep Length and Elapsed Duration (N & dt)

The timested length N and elapsed duration dt were chosen such that the ipopt solver would solve the equation in a reasonable amount of time. The time step of 0.1 means we're predicting 1 second into the future which isn't too long as to accrue a large error in the predicted path and 0.1s also happens to be equivalent to the latency before actuator changes. Using a N=25 as in the lesson with a 0.05 timestep takes way too long to solve. The timestep chosen was suggested in the Udacity Q&A for this project.

## Polynomial Fitting and MPC Preprocessing

The vehicle's position and path points were preprocessed to place the car at the origin, making later calculations much simpler. This is seen in `main.cpp` in the following lines:

```c++
for (size_t i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 90 degrees
    double shift_x = ptsx[i]-px;
    double shift_y = ptsy[i]-py;

    ptsx[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
    ptsy[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
}
```

## Model Predictive Control with Latency

The initial state is calculated such that we use the car's position 100ms into the future instead of it's initial state to feed into the solver. The calculations for this can be seen in `main.cpp` in the following lines:

```c++
// Estimate future state
double future_x = v * cos(steer_value) * latency;
double future_y = v * sin(steer_value) * latency;
double future_psi = v * steer_value * latency / Lf;
v += throttle_value * latency;  // using throttle as an estimation of acceleration
cte += v * sin(epsi) * latency;
epsi += v * epsi * latency / Lf;

Eigen::VectorXd state(6);
state << future_x, future_y, future_psi, v, cte, epsi;
```
