# # Example 8
# This example is from the book Decomposition Techniques in Mathematical Programming
# Chapter 7.2, page 281, [url](https://www.springer.com/gp/book/9783540276852)

# Bold points in example: Here, we are using upperonly, loweronly for defining variables. Also, we are using equality in constraints. Using upperonly/loweronly will create variables that will appear in the proper model. By this definition, we might have slightly smaller reformulated models in the algorithm. 

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

using BilevelJuMP
using Ipopt
using JuMP
using Test

model = BilevelModel(Ipopt.Optimizer; mode = BilevelJuMP.ProductMode(1e-9))

# First we need to create all of the variables in the upper and lower problems:

# Upper level variables
@variable(Upper(model), x)
@variable(UpperOnly(model), z)

# Lower level variables
@variable(Lower(model), y)
@variable(LowerOnly(model), w)

# Then we can add the objective and constraints of the upper problem:

# Upper level objecive function
@objective(Upper(model), Min, -x + 4y + z)

# Upper level constraints
@constraint(Upper(model), y + 2x + z <= 9)
@constraint(Upper(model), z == 1)

# Followed by the objective and constraints of the lower problem:

# Lower objective function
@objective(Lower(model), Min, -x - y + w)

# Lower constraints
@constraint(Lower(model), y >= 0)
@constraint(Lower(model), x + y + w <= 8)
@constraint(Lower(model), x >= 0)
@constraint(Lower(model), x <= 4)
@constraint(Lower(model), w == 1)

# Initial Starting conditions  #src

# Now we can solve the problem and verify the solution again that reported by book

optimize!(model)

# Auto testing
@test value(x) ≈ 1 atol = 1e-2
@test value(y) ≈ 6 atol = 1e-2
@test value(z) ≈ 1 atol = 1e-2
@test value(w) ≈ 1 atol = 1e-2
