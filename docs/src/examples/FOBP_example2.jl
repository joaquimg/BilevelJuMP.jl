# # Foundations of Bilevel Programming: Example 2

# TODO: add a link to the book  #src

# This example is from the book _Foundations of Bilevel Programming_ by Stephan
# Dempe, Chapter 3.2, Page 25.

# TODO:                                                             #src
#   We should include the math formulation here, or at least some   #src
#   description of the problem.                                     #src

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

@constraints(Upper(model), begin
    u1, x <= 5
    u2, y <= 8
    u3, y >= 0
end)

# Followed by the objective and constraints of the lower problem:

@objective(Lower(model), Min, -x)

#-

@constraint(Lower(model), l1,  x +  y <= 8)
@constraint(Lower(model), l2, 4x +  y >= 8)
@constraint(Lower(model), l3, 2x +  y <= 13)
@constraint(Lower(model), l4, 2x - 7y <= 0)

# !!! tip
#     You can use the singular `@constraint` macro or the plural `@constraints`!

# We can also set hints for the variables associated with the problems.

# In this example, we know the duals on the lower constraints are in
# the set `[-15, 15]`:
for c in [l1, l2, l3, l4]
    BilevelJuMP.set_dual_upper_bound(c, 15)
    BilevelJuMP.set_dual_lower_bound(c, -15)
end

# While we think the primal variables are in `[-10, 6]` for `x` and `[-1, 9]`
# for `y`. These hints are optional. But supplying them (e.g., from domain
# knowledge) can be helpful for the solver.

BilevelJuMP.set_primal_lower_bound_hint(x, -10)
BilevelJuMP.set_primal_upper_bound_hint(x, 6)
BilevelJuMP.set_primal_lower_bound_hint(y, -1)
BilevelJuMP.set_primal_upper_bound_hint(y, 9)

# Now we can solve the problem and verify the solution again that reported by
# Dempe.

optimize!(model)

#-

@test objective_value(model) ≈ 3 * (3.5 * 8 / 15) + (8 / 15) atol=1e-6
@test BilevelJuMP.lower_objective_value(model) ≈ -3.5 * 8 / 15 atol=1e-6
@test objective_value(Lower(model)) ≈ -3.5 * 8 / 15 atol=1e-6
@test value(x) ≈ 3.5 * 8 / 15 atol=1e-6
@test value(y) ≈ 8 / 15 atol=1e-6
@test value(u1) ≈ 3.5 * 8 / 15 atol=1e-6
@test value(l1) ≈ 4.5 * 8 / 15 atol=1e-6
@test dual(l1) ≈ [0] atol=1e-6
@test dual(l3) ≈ [0] atol=1e-6

# TODO: why are these commented out?    #src
# @test dual(l2) #≈ [0] atol=atol       #src
# @test dual(l4) #≈ [0] atol=atol       #src
