# # Foundations of Bilevel Programming: Example Chapter 5.1, Page 127
#
# This example is from the book _Foundations of Bilevel Programming_ by Stephan
# Dempe, Chapter 5.1, Page 127. [url](https://www.springer.com/gp/book/9781402006319)

# Model of the problem

# First level
# ```math
# \min x^2 + y,\\
# \notag s.t.\\
# -x-y\leq 0,\\
# ```
# Second level
# ```math
# \min x,\\
# \notag s.t.\\
# x \geq 0,\\
# ```

using BilevelJuMP
using Ipopt
using Test #src

model = BilevelModel(Ipopt.Optimizer; mode = BilevelJuMP.ProductMode(1e-9))

# First we need to create all of the variables in the upper and lower problems:

# Upper level variables
@variable(Upper(model), y, start = 0)

#Lower level variables
@variable(Lower(model), x, start = 0)

# Then we can add the objective and constraints of the upper problem:

# Upper level objecive function
@objective(Upper(model), Min, x^2 + y)

# Upper level constraints
@constraint(Upper(model), u1, -x - y <= 0)

# Followed by the objective and constraints of the lower problem:

# Lower objective function
@objective(Lower(model), Min, x)

# Lower constraints
@constraint(Lower(model), l1, x >= 0)

# Initial Starting conditions  #src

# Now we can solve the problem and verify the solution again that reported by
# Dempe.

optimize!(model)

#

primal_status(model)

#

termination_status(model)

# Results

objective_value(model)

#

value(x)

#

value(y)

# Auto testing #src
atol = 1e-3 # src
@test objective_value(model) ≈ 0 atol = atol #src
@test value(x) ≈ 0 atol = atol #src
@test value(y) ≈ 0 atol = atol #src
