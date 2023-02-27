# # Foundations of Bilevel Programming: Example 5
# This example is from the book _Foundations of Bilevel Programming_ by Stephan
# Dempe, Chapter 8.1, Page 255. [url](https://www.springer.com/gp/book/9781402006319)

# Here, only the second level is described

# Model of the problem
# First level
# ```math
# \min 0,\\
# ```
# Second level
# ```math
# \min x,\\
# \notag s.t.\\
# x+y \leq 2,\\
# x-y \leq 2,\\
# -4x+5y \leq 10,\\
# -4x-5y \leq 10,\\
# ```

using BilevelJuMP
using Ipopt
using JuMP
using Test

model = BilevelModel(Ipopt.Optimizer; mode = BilevelJuMP.ProductMode(1e-9))

# Global variables
atol = 1e-3

# First we need to create all of the variables in the upper and lower problems:

# Upper level variables
@variable(Upper(model), y, start = 0)

#Lower level variables
@variable(Lower(model), x, start = 0)

# Then we can add the objective and constraints of the upper problem:

# Upper level objecive function
@objective(Upper(model), Min, 0 * y + 0)

# Followed by the objective and constraints of the lower problem:

# Lower objective function
@objective(Lower(model), Min, x)

# Lower constraints
@constraint(Lower(model), x + y <= 2)
@constraint(Lower(model), x - y <= 2)
@constraint(Lower(model), -4x + 5y <= 10)
@constraint(Lower(model), -4x - 5y <= 10)

# Initial Starting conditions  #src

# Now we can solve the problem and verify the solution again that reported by
# Dempe.

optimize!(model)
primal_status(model)
termination_status(model)
