# # Example 1 on Iterative Solution for ProductMode
# This example is again from the book Decomposition Techniques in Mathematical Programming
# Chapter 7.2, page 281, [url](https://www.springer.com/gp/book/9783540276852)
# The formulation is the same as used before, but we are now focussing on more practical aspects of an iterative product mode.
# In particular, a few tricks for solving iteratively with Ipopt are provided. 

# Model of the problem
# First level
# ```math
# \min -x + 4y + z,\\
# \notag s.t.\\
# y + 2x + z \leq 9,\\
# z = 1,\\
# ```
# Second level
# ```math
# \min -x - y + w,\\
# \notag s.t.\\
# y \geq 0,\\
# x + y + w \leq 8,\\
# x \geq 0,\\
# x \leq 4,\\
# x = 1.\\
# ```

using JuMP, BilevelJuMP, Ipopt

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
model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-0;IterEps=IterEps, IterAttr = IterAttr))

# The following lines are copied from the example as presented before: 
@variable(Upper(model), x)
@variable(UpperOnly(model), z)
@variable(Lower(model), y)
@variable(LowerOnly(model), w)
@objective(Upper(model), Min, -x + 4y + z)
@constraint(Upper(model), y + 2x + z <= 9)
@constraint(Upper(model), z == 1)
@objective(Lower(model), Min, -x - y + w)
@constraint(Lower(model),  y >= 0)
@constraint(Lower(model), x + y + w <= 8)
@constraint(Lower(model),  x >= 0)
@constraint(Lower(model),  x <= 4)
@constraint(Lower(model),  w == 1)

# Setting the right solver options for NLPs can make a huge difference. It is recommended to play around with these when solving challenging models.
# All options set with set_optimizer_attribute() carry over to subsequent iterations. 
# In case you want to return to default settings for following solves, you have to manually specify these in the IterAttr options again. 
# For the initial solution, we want to solve with a target mu of 1e-9 (see the Ipopt docs for an explanation), which is the initial mu as set in IterAttr. 
# Other options can easily be set the same way. 
set_optimizer_attribute(model, "mu_target", 1e-9)

# Warmstarting the first iteration is also possible, start values need to be provided using set_start_value() and set_dual_start_value() functions, or in the variable declarations.
# Note, that if you are using starting values, the solver options must be set accordingly for the first iteration (as above...).

# Finally, we can optimize: 
optimize!(model)

# Auto testing
@test value(x) ≈ 1 atol=1e-4
@test value(y) ≈ 6 atol=1e-4
@test value(z) ≈ 1 atol=1e-4
@test value(w) ≈ 1 atol=1e-4
