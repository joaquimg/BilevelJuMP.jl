# # Example 2 on Iterative Solution for ProductMode
# This example is from the paper S. Siddiqui & S. A. Gabriel (2012): "An SOS1-Based Approach for Solving MPECs with a Natural Gas Market Application", as appearing in "Networks and Spatial Economics"
# It can be found in section 3 "Numerical Examples"

# The leader solves the following first level
# ```math
# \max_{Q \geq 0} (a-b(q_1 + q_2 + Q))Q + CQ,\\
# ```
# Firm i = 1,2 solves the following lower-level problem where it takes the upper-level quantity Q as fixed and tries to maximize profits while in Nash-Cournot competition with the other Stackelberg follower firm j.
# ```math
# \max_{q_i \geq 0} (a-b(q_i + q_j + Q))q_i + c_i q_i,\\
# ```
# Subsequently, we will combine the two lower level players' problem into a single one with equivalent KKTs. 

using JuMP, BilevelJuMP, Ipopt

F = [1,2]
c = Dict(1=>1, 2=>1)
C = 1
a = 13
b = 1

# First, we need to specify an iterable with our desired regularization values.
# The problem will be solved for each element in this series, while the next step starts at the optimal values of the previous iteration. 
# Note, that this iterable specifies regularization parameters after a first solve with the initial value 1e-0. 
IterEps = [1e-0 * 0.99^i for i in 1:1500]

# Also, it is possible to give specific solver attributes that are not valid for the initial solve, but only subsequent ones.
# Warmstarting interior point algorithms is difficult, but some of the following options can be recommended. 
# Allowing warmstarts in Ipopt explicitly is necessary in Ipopt, this may vary with different solvers.
# Also, we are suppressing solver outputs for all but the first iteration (and could instead write a log to a file using the commented lines).

IterAttr = Dict( 
    "mu_init" => 1e-9, 
    "warm_start_init_point"=> "yes", 
    "warm_start_bound_push"=> 1e-12, 
    "warm_start_bound_frac"=> 1e-12, 
    "warm_start_slack_bound_frac"=> 1e-12, 
    "warm_start_slack_bound_push"=> 1e-12, 
    "warm_start_mult_bound_push" => 1e-12, 
    # "output_file" => "solver.log",
    # "file_print_level" => 8,
    "print_level" => 0,
    )
# Note that for other solvers, these options may differ. 
# You can also specify other options here. 
# For example, you could play around with an adaptive mu strategy in the initial solve and a monotone mu strategy in subsequent iterations (see the Ipopt options docs). 

# We can now specify our model, where the initial regularization is 1e+0, followed by setting the iterative values and settings:
model1 = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-0;IterEps=IterEps, IterAttr = IterAttr))

# The following lines are copied from the example as presented before: 
@variable(Lower(model1), q[F] >= 0)
@variable(Upper(model1), Q >= 0)

@objective(Upper(model1), Max, ((a-b * (q[1] + q[2] + Q)) * Q - C*Q) )

@objective(Lower(model1), Min, -((a-b * (q[1] + q[2] + Q)) * q[1] - C*q[1] + (a-b * (q[1] + q[2] + Q)) * q[2] - C*q[2] + b*q[1]*q[2]) )

# Setting the right solver options for NLPs can make a huge difference. It is recommended to play around with these when solving challenging models.
# All options set with set_optimizer_attribute() carry over to subsequent iterations. 
# In case you want to return to default settings for following solves, you have to manually specify these in the IterAttr options again. 
# For the initial solution, we want to solve with a target mu of 1e-9 (see the Ipopt docs for an explanation), which is the initial mu as set in IterAttr. 
# Other options can easily be set the same way. 
set_optimizer_attribute(model1, "mu_target", 1e-9)

# Warmstarting the first iteration is also possible, start values need to be provided using set_start_value() and set_dual_start_value() functions, or in the variable declarations.
# Note, that if you are using starting values, the solver options must be set accordingly for the first iteration (as above...).

# Finally, we can optimize: 
optimize!(model1)

# To verify that the KKTs are correctly set, we could also print the model to an lp file: 
# optimize!(model1, bilevel_prob = "blp.lp")

# Auto testing

@test isapprox(value(model1[:Q]), 6; atol=1e-6)
@test isapprox(value.(model1[:q]).data, [2,2]; atol=1e-6) 

#######################
# The following lines show the other two datasets provided in the original paper. 
# Dataset 2: 
F = [1,2]
c = Dict(1=>1, 2=>1)
C = 1
a = 13
b = 0.1

IterEps = [1e-0 * 0.99^i for i in 1:1500]

IterAttr = Dict( 
    "mu_init" => 1e-9, 
    "warm_start_init_point"=> "yes", 
    "warm_start_bound_push"=> 1e-12, 
    "warm_start_bound_frac"=> 1e-12, 
    "warm_start_slack_bound_frac"=> 1e-12, 
    "warm_start_slack_bound_push"=> 1e-12, 
    "warm_start_mult_bound_push" => 1e-12, 
    "print_level" => 0,
    )

model2 = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-0;IterEps=IterEps, IterAttr = IterAttr))

@variable(Lower(model2), q[F] >= 0)
@variable(Upper(model2), Q >= 0)

@objective(Upper(model2), Max, ((a-b * (q[1] + q[2] + Q)) * Q - C*Q) )

@objective(Lower(model2), Min, -((a-b * (q[1] + q[2] + Q)) * q[1] - C*q[1] + (a-b * (q[1] + q[2] + Q)) * q[2] - C*q[2] + b*q[1]*q[2]) )

set_optimizer_attribute(model2, "mu_target", 1e-9)

optimize!(model2)

# Auto testing

@test isapprox(value(model2[:Q]), 60; atol=1e-6)
@test isapprox(value.(model2[:q]).data, [20,20]; atol=1e-6) 

#######################
# Dataset 3: 
F = [1,2]
c = Dict(1=>1, 2=>1)
C = 2
a = 13
b = 0.1

IterEps = [1e-0 * 0.99^i for i in 1:1500]

IterAttr = Dict( 
    "mu_init" => 1e-9, 
    "warm_start_init_point"=> "yes", 
    "warm_start_bound_push"=> 1e-12, 
    "warm_start_bound_frac"=> 1e-12, 
    "warm_start_slack_bound_frac"=> 1e-12, 
    "warm_start_slack_bound_push"=> 1e-12, 
    "warm_start_mult_bound_push" => 1e-12, 
    "print_level" => 0,
    )

model3 = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-0;IterEps=IterEps, IterAttr = IterAttr))

@variable(Lower(model3), q[F] >= 0)
@variable(Upper(model3), Q >= 0)

@objective(Upper(model3), Max, ((a-b * (q[1] + q[2] + Q)) * Q - C*Q) )

@objective(Lower(model3), Min, -((a-b * (q[1] + q[2] + Q)) * q[1] - C*q[1] + (a-b * (q[1] + q[2] + Q)) * q[2] - C*q[2] + b*q[1]*q[2]) )

set_optimizer_attribute(model3, "mu_target", 1e-9)

optimize!(model3)

# Auto testing

@test isapprox(value(model3[:Q]), 55; atol=1e-6)
@test isapprox(value.(model3[:q]).data, [18.333,18.333]; atol=1e-2) 