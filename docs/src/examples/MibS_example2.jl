# # MibS: Example 2 (Experimental feature)

# Model of the problem
# First level
# ```math
# \min_{x} 2x -4y + 10z,\\
# \notag s.t.\\
# -3x + 2y + 2z \leq 12,\\
# x + 2y \leq 20,\\
# x \leq 10,\\
# x \in \mathbb{Z}, z \in \mathbb{B},\\
# ```
# Second level
# ```math
# \min_{y} y,\\
# \notag s.t.\\
# 2x - y + 3z<= 7,\\
# -2x + 4y <= 16,\\
# y <= 5\\
# y \in \mathbb{Z}\\
# ```

using BilevelJuMP
using Test
using MibS_jll

# MibS is an external solver rather than a JuMP optimizer, so it is selected
# with a `mode` and needs no `set_optimizer`. `MibS_jll` is not a dependency of
# BilevelJuMP, so its executable is passed in explicitly.

model = BilevelModel()
BilevelJuMP.set_mode(model, BilevelJuMP.MibSMode(MibS_jll.mibs))

# First we need to create all of the variables in the upper and lower problems:

# Upper level variables
@variable(Upper(model), x, Int)
@variable(Upper(model), z, Bin)

#Lower level variables
@variable(Lower(model), y, Int)

# Then we can add the objective and constraints of the upper problem:

# Upper level objective function
@objective(Upper(model), Min, 2x - 4y + 10z)

# Upper constraints
@constraints(Upper(model), begin
    u1, -3x + 2y + 5z <= 12
    u2, x + 2y <= 20
    u3, x <= 10
end)

# Followed by the objective and constraints of the lower problem:

# Lower objective function
@objective(Lower(model), Min, y)

# Lower constraints
@constraint(Lower(model), l1, 2x - y + 3z <= 7)
@constraint(Lower(model), l2, -2x + 4y <= 16)
@constraint(Lower(model), l3, y <= 5)

# Now we can solve the problem and query the solution with the usual JuMP
# functions:

optimize!(model)

termination_status(model)

objective_value(model)

value(x)

value(z)

value(y)

# Auto testing
@test termination_status(model) == MOI.OPTIMAL
@test primal_status(model) == MOI.FEASIBLE_POINT
@test objective_value(model) ≈ -8.0
@test value(x) ≈ 6.0
@test value(z) ≈ 0.0
@test value(y) ≈ 5.0
