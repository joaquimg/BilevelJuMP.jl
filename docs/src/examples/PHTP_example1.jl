# # Princeton Handbook of Test Problems: Test 9.3.2
#
# This example is from the book Princeton Handbook of Test Problems in Local
# and Global Optimization, Floudas et al., Chapter 9.3.2, Page 221,
# [url](https://www.springer.com/gp/book/9780792358015).

# Model of the problem
# First level
# ```math
# \min (x-5)^2+(2y+1)^2,\\
# \notag s.t.\\
# x \geq 0,\\
# y \geq 0,\\
# ```
# Second level
# ```math
# \min (y-1)^2-1.5xy,\\
# \notag s.t.\\
# -3x+y \leq -3,\\
# x-0.5y \leq 4,\\
# x+y \leq 7,\\
# ```

using BilevelJuMP
using Ipopt
using Test #src

model = BilevelModel(Ipopt.Optimizer; mode = BilevelJuMP.ProductMode(1e-9))

# First we need to create all of the variables in the upper and lower problems:

# Upper level variables
@variable(Upper(model), x)

#Lower level variables
@variable(Lower(model), y)

# Then we can add the objective and constraints of the upper problem:

# Upper level objecive function
@objective(Upper(model), Min, (x - 5)^2 + (2y + 1)^2)

# Upper level constraints
@constraint(Upper(model), x >= 0)
@constraint(Upper(model), y >= 0)

# Followed by the objective and constraints of the lower problem:

# Lower objective function
@objective(Lower(model), Min, (y - 1)^2 - 1.5 * x * y)

# Lower constraints
@constraint(Lower(model), -3x + y <= -3)
@constraint(Lower(model), x - 0.5y <= 4)
@constraint(Lower(model), x + y <= 7)


# Now we can solve the problem and verify the solution again that reported by
# the book.

optimize!(model)

#

primal_status(model)

#

termination_status(model)

#

value(x)

#

value(y)

atol = 1e-3 #src
@test value(x) ≈ 1 atol = atol #src
@test value(y) ≈ 0 atol = atol #src
