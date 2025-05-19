# # Decomposition Techniques in Mathematical Programming: Example 7.4
#
# This example is from the book Decomposition Techniques in Mathematical Programming
# Chapter 7.2, page 281, [url](https://www.springer.com/gp/book/9783540276852)


# Model of the problem
# First level
# ```math
# \min -4y - x,\\
# \notag s.t.\\
# y + 2x \leq 8,\\
# ```
# Second level
# ```math
# \min - y - x,\\
# \notag s.t.\\
# - y \leq 0,\\
# y + x \leq 7,\\
# - x \leq 0,\\
# x \leq 4,\\
# ```

using BilevelJuMP
using Ipopt
using Test #src

model = BilevelModel(Ipopt.Optimizer; mode = BilevelJuMP.ProductMode(1e-5))

set_silent(model)

# First we need to create all of the variables in the upper and lower problems:

# Upper level variables
@variable(Upper(model), x, start = 1)

# Lower level variables
@variable(Lower(model), y, start = 6)

# Then we can add the objective and constraints of the upper problem:

# Upper level objecive function
@objective(Upper(model), Min, 4y - x)

# Upper level constraints
@constraint(Upper(model), y + 2x <= 8)

# Followed by the objective and constraints of the lower problem:

# Lower objective function
@objective(Lower(model), Min, - y -x )

# Lower constraints
@constraint(Lower(model), - y <= 0)
@constraint(Lower(model), y + x <= 7)
@constraint(Lower(model), - x <= 0)
@constraint(Lower(model), x <= 4)

# Now we can solve the problem and verify the solution again that reported by book

optimize!(model)

# Results

value(x)

#

value(y)

@test value(x) ≈ 1 atol = 1e-2 #src
@test value(y) ≈ 6 atol = 1e-2 #src
