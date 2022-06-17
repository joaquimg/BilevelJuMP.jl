# # Example 9
# This example is from the book Investment in Electricity Generation and Transmission. [url](https://www.springer.com/gp/book/9783319294995)
# Bold point(s): using Dualof() in case we have information on the dual variable(s) of the lower level problem. 


# Model of the problem 
# First level
# ```math
# \min 40000x + 8760*(10y_1-\lambda * y_1),\\
# \notag s.t.\\
# 0 \leq x \leq 250,\\
# 0 \leq \lambda \leq 20,\\
# ```
# Second level
# ```math
# \min 10y_1 + 12y_2 + 15y_3,\\
# \notag s.t.\\
# y_1 + y_2 + y_3 = 200,\quad [\lambda]\\
# y_1 \leq x,\\
# y_2 \leq 150,\\
# y_3 \leq 100,\\
# y[i] \geq 0, \forall i \in I\\
# ```
# In this problem, ``\lambda`` in upper level objective function is dual variable of the first constraint is the lower level problem.

using BilevelJuMP
using Ipopt
using JuMP
using Test

model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-9))

# First we need to create all of the variables in the upper and lower problems:

# Upper level variables
@variable(Upper(model), x, start = 50)

# Lower level variables
@variable(Lower(model), -1 <= y[i=1:3] <= 300, start = [50, 150, 0][i])

# Then we can add the constraints of the upper problem:
@constraint(Upper(model), lb0, x >= 0)
@constraint(Upper(model), ub0, x <= 250)

# Followed by the constraints of the lower problem:

# Right after the definition of a constraint, we are able to define its dual variable. In order to this, we need a reference to the constraint. Here **b** is a reference to the constraint with the dual variable  ``\lambda``.
@constraint(Lower(model), b, y[1] + y[2] + y[3] == 200)
@variable(Upper(model), 0 <= lambda <= 20, DualOf(b), start = 15)

@constraint(Lower(model), ub1, y[1] <= x)
@constraint(Lower(model), ub2, y[2] <= 150)
@constraint(Lower(model), ub3, y[3] <= 100)
@constraint(Lower(model), lb[i=1:3], y[i] >= 0)

# At last we have objective function of the upper and lower problem:
@objective(Upper(model), Min, 40_000x + 8760*(10y[1]-lambda*y[1]))
@objective(Lower(model), Min, 10y[1] + 12y[2] + 15y[3])

# Now we can solve the problem and verify the solution again that reported by book:
optimize!(model)
primal_status(model)
termination_status(model)

# Auto testing:
@test objective_value(model) ≈ -190_000 atol=1e-1 rtol=1e-2
@test value(x) ≈ 50 atol=1e-3 rtol=1e-2
@test value.(y) ≈ [50, 150, 0] atol=1e-3 rtol=1e-2
@test value(lambda) ≈ 15 atol=1e-3 rtol=1e-2
