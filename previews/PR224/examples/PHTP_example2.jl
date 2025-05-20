# # Princeton Handbook of Test Problems: Test 9.3.4
#
# This example is from the book Princeton Handbook of Test Problems in Local and Global Optimization
# Dempe, Chapter 9.3.4 -parg 223 [url](https://www.springer.com/gp/book/9780792358015)

# Here, only the second level is described

# Model of the problem
# First level
# ```math
# \min 2x_1+2x_2-3y_1-3y_2-60,\\
# \notag s.t.\\
# x_1 + x_2 + y_1 -2y_2 -40\leq 0,\\
# 0 \leq x_i \leq 50, \forall i \in I,\\
# -10 \leq y_j \leq 20, \forall j \in J,\\
# ```
# Second level
# ```math
# \min (-x_1 + y_1 + 40)^2 + (-x_2 + y_2 + 20)^2,\\
# \notag s.t.\\
# - x_i + 2y_j <= -10,\forall (i,j) \in \{(i,j)|i\in I, j\in J, i=j\},\\
# -10 \leq y_j \leq 20, \forall j \in J.\\
# ```

using BilevelJuMP
using Ipopt
using Test #src

model = BilevelModel(Ipopt.Optimizer; mode = BilevelJuMP.ProductMode(1e-9))

# First we need to create all of the variables in the upper and lower problems:

# Upper level variables
@variable(Upper(model), x[i = 1:2], start = 0)

# Lower level variables
@variable(Lower(model), y[i = 1:2], start = -10)

# Then we can add the objective and constraints of the upper problem:

# Upper level objecive function
@objective(Upper(model), Min, 2x[1] + 2x[2] - 3y[1] - 3y[2] - 60)

# Upper level constraints
@constraint(Upper(model), x[1] + x[2] + y[1] - 2y[2] - 40 <= 0)
@constraint(Upper(model), [i = 1:2], x[i] >= 0)
@constraint(Upper(model), [i = 1:2], x[i] <= 50)
@constraint(Upper(model), [i = 1:2], y[i] >= -10)
@constraint(Upper(model), [i = 1:2], y[i] <= 20)

# Followed by the objective and constraints of the lower problem:

# Lower objective function
@objective(Lower(model), Min, (-x[1] + y[1] + 40)^2 + (-x[2] + y[2] + 20)^2)

# Lower constraints
@constraint(Lower(model), [i = 1:2], -x[i] + 2y[i] <= -10)
@constraint(Lower(model), [i = 1:2], y[i] >= -10)
@constraint(Lower(model), [i = 1:2], y[i] <= 20)

# Now we can solve the problem and verify the solution again that reported by the book

optimize!(model)

#

primal_status(model)

#

termination_status(model)

#

objective_value(model)

#

value.(x)

#

value.(y)

# Auto testing #src
@test objective_value(model) ≈ 0 atol = 1e-3 #src
sol = vcat(value.(x), value.(y)) #src
@test sol ≈ [0; 0; -10; -10] || sol ≈ [0; 30; -10; 10] #atol=1e-3 #src

# # Like any other optimization problem, there is a chance in bilevel
# optimization to find multiple solutions with the same optimal value;
# based on the inherent stochasticity of the algorithm and random seed,
# we are expecting two optimal solutions for this problem. 
