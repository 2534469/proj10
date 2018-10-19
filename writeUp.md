# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

##The Model

In this project we use kinematic model, which do not take the gravity, 
air resistance and tire forces, but are pretty tractable.
The state of the car is described as a vector including:
1. Vehicle position x
2. Vehicle position y
3. psi - vehicle orientation
4. v - car velocity
5. cte, the cross track error, distance to the trajectory of the car
6. epsi, the error in the angle between the predicted vehicle orientation and trajectory orientation.

The control inputs are delta, the steering angle and a, a throttle and break value.
Delta can be between [-25,25] degrees and a values lie in [-1,1]
The calculated state from the previous can be predicted by a model, using the state and 
actuators value:
   
```cpp
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] =cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```
Note, the left hand side should equals 0.

The cost function is defined as a sum of errors discrepances over the caluculated timestamps
and include:
1. Squared cte
2. Squared psi error
3. Suquared difference between reference speed and the predicted one
4. Sum of squared magnitudes of a and delta, aiming minimizing their usage
5. Sum of squared difference of a and delta between consecutive timestamps. 

```cpp
cons double cte_coeff = 15000;
const double epsi_coeff = 10000;
const double delta_coeff = 100;
const double a_coeff = 20;
const double delta_diff_coeff = 100;

for (int t = 0; t < N; t++) {
    fg[0] += cte_coeff*CppAD::pow(vars[cte_start + t] - ref_cte, 2); //cte_coeff
    fg[0] += epsi_coeff*CppAD::pow(vars[epsi_start + t] - ref_epsi, 2); //epsi_coeff
    fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
}
// Minimize the use of actuators.
for (int t = 0; t < N - 1; t++) {
    fg[0] += delta_coeff*CppAD::pow(vars[delta_start + t], 2);
    fg[0] += a_coeff*CppAD::pow(vars[a_start + t], 2);
}
// Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++) {
    fg[0] += delta_diff_coeff*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
    fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}


```

##Timestep Length and Elapsed Duration (N & dt)

For the number of steps, N I've chosen the value N=10 and time between steps dt = 0.1.
If one choses N too large, say N =100, the processing time takes too long and the car gets 
out of the track, since the model reacts too slow. For small number a polynomial fit leads to
faulty results. Small dt also lead to increase of the processing time, and as result to 
oscillation of the car or flying out of the track, since the actuators receive their values 
to late.
 

##Polynomial Fitting and MPC Preprocessing

For the polynomial fitting I first translate coordination to the vehicle coordination system
by shifting and rotating, and also negate the angle since simulator turns right for positive 
values of steering angle, and not left. 
```cpp
vector<double> ptsx_new;
vector<double> ptsy_new;
for(int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - px;
    double shift_y = ptsy[i] - py;
    double ptx_new = shift_x * cos(-psi) - shift_y * sin(-psi); //delta anticlockwise
    double pty_new = shift_x * sin(-psi) + shift_y * cos(-psi);
    ptsx_new.push_back(ptx_new);
    ptsy_new.push_back(pty_new);
}
```

##Model Predictive Control with Latency

A latency of 100 ms corresponds to one timestep in the model implementation.
To dump a latency I also apply the actuators one time step later, see MPC.cpp(line 109-115).
Also for a cost function I penalize the cte error and epsi error together with 
usage of actuators and delta difference, trying to make the behavior of the car more 
smooth on the corners.
