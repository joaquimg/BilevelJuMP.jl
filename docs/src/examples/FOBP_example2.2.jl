# Foundations of Bilevel Programming: Example 2.1

# link to the book https://www.springer.com/gp/book/9781402006319

# This example is from the book _Foundations of Bilevel Programming_ by Stephan
# Dempe, Chapter 3.2, Page 25.
# Moving the bound on x to lower level

#------------------------------------------------------------------
#------------------------------------------------------------------
# Model of the problem
#------------------------------------------------------------------
# First level
# Min 3x + y 
# s.t.
# 0 <= y <=8
#------------------------------------------------------------------
# Second level
# Min -x
# s.t.
# x + y <= 8
# 4x + y >= 8
# 2x + y <= 13
# 2x - y <= 0
# x <= 5
#------------------------------------------------------------------
#------------------------------------------------------------------


using BilevelJuMP
using Ipopt
using JuMP
using Test

model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-9))

# First we need to create all of the variables in the upper and lower problems:

@variable(Lower(model), x, start = 3.5 * 8 / 15)

#-

@variable(Upper(model), y, start = 8 / 15)

# Then we can add the objective and constraints of the upper problem:

@objective(Upper(model), Min, 3x + y)
#-

@constraint(Upper(model), y <= 8)
@constraint(Upper(model), y >= 0)

# Followed by the objective and constraints of the lower problem:

@objective(Lower(model), Min, -x)

#-
@constraint(Lower(model), x +  y <= 8)
@constraint(Lower(model), 4x +  y >= 8)
@constraint(Lower(model), 2x +  y <= 13)
@constraint(Lower(model), 2x - 7y <= 0)
@constraint(Lower(model), x <= 5)

# Now we can solve the problem and verify the solution again that reported by
# Dempe.

optimize!(model)

@test objective_value(model) ≈ 3 * (3.5 * 8 / 15) + (8 / 15) atol=1e-6\
@test value(x) ≈ 3.5 * 8 / 15 atol=1e-6
@test value(y) ≈ 8 / 15 atol=1e-6

# TODO: why are these commented out?    #src
# @test dual(l2) #≈ [0] atol=atol       #src
# @test dual(l4) #≈ [0] atol=atol       #src
