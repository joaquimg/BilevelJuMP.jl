# # MibS: Example 1 (Experimental feature)

# Model of the problem
# First level
# ```math
# \min_{x} -3x -7y,\\
# \notag s.t.\\
# -3x + 2y \leq 12,\\ 
# x + 2y \leq 20,\\
# x \leq 10,\\
# x \in \mathbb{Z},\\
# ```
# Second level
# ```math
# \min_{y} y,\\
# \notag s.t.\\
# 2x - y <= 7,\\
# -2x + 4y <= 16,\\
# y <= 5\\
# y \in \mathbb{Z}\\
# ```

using BilevelJuMP
using Test
using MibS_jll

model = BilevelModel()

# First we need to create all of the variables in the upper and lower problems:

# Upper level variables
@variable(Upper(model), x, Int)

#Lower level variables
@variable(Lower(model), y, Int)

# Then we can add the objective and constraints of the upper problem:

# Upper level objecive function
@objective(Upper(model), Min, -3x - 7y)

# Upper constraints
@constraints(Upper(model), begin
    u1, -3x + 2y <= 12
    u2, x + 2y <= 20
    u3, x <= 10
end)

# Followed by the objective and constraints of the lower problem:

# Lower objective function
@objective(Lower(model), Min, y)

# Lower constraints
@constraint(Lower(model), l1, 2x - y <= 7)
@constraint(Lower(model), l2, -2x + 4y <= 16)
@constraint(Lower(model), l3, y <= 5)

# Using MibS Solver
solution = BilevelJuMP.solve_with_MibS(model, MibS_jll.mibs)

# Auto testing
@test solution.status == true
@test solution.objective ≈ -53
@test solution.nonzero_upper == Dict(0 => 6.0)
@test solution.nonzero_lower == Dict(0 => 5.0)
@test solution.all_upper["x"] == 6.0
@test solution.all_lower["y"] == 5.0
