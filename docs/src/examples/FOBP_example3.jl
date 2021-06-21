# Foundations of Bilevel Programming: Example 3

# link to the book https://www.springer.com/gp/book/9781402006319

# This example is from the book _Foundations of Bilevel Programming_ by Stephan
# Dempe, Chapter 3.4.1, Page 32.

#------------------------------------------------------------------
#------------------------------------------------------------------
# Model of the problem
#------------------------------------------------------------------
# First level
# Min 2*x[2] - y 
# s.t.
# y >= 0
# y <= 3
#------------------------------------------------------------------
# Second level
# Min -x[1]
# s.t.
# 100*x[1] - x[2] <= 1
# x[2] <= y
# x[1] >= 0
# x[2] >= 0
#------------------------------------------------------------------
#------------------------------------------------------------------


using BilevelJuMP
using Ipopt
using JuMP
using Test

model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-9))

# First we need to create all of the variables in the upper and lower problems:

@variable(Lower(model), x[i=1:2])

#-

@variable(Upper(model), y)

# Then we can add the objective and constraints of the upper problem:

@objective(Upper(model), Min, 2x[2] - y)

#-

@constraint(Upper(model), y <= 3)
@constraint(Upper(model), y >= 0)

# Followed by the objective and constraints of the lower problem:

@objective(Lower(model), Min, -x[1])

#-

@constraint(Lower(model),  100x[1] -x[2] <= 1)
@constraint(Lower(model), x[2] <= y)
@constraint(Lower(model), x[1] >= 0)
@constraint(Lower(model), x[2] >= 0)

# Now we can solve the problem and verify the solution again that reported by
# Dempe.

optimize!(model)
primal_status(model)
termination_status(model)

@test objective_value(model) ≈ 0 atol=1e-5

@test value(x[1]) ≈ 1/100 atol=atol
@test value(x[2]) ≈ 0 atol=atol
@test value(y) ≈ 0 atol=1e-5


