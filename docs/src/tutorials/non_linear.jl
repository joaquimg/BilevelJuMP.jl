# # Non Linear models

# BilevelJuMP has limited support for non-linear models.
# The `@NLconstraint` and the `@NLobjective` macros are supported
# for the upper level, but not for the lower level.
# Moreover, these macros can only be used if the selected solver
# supports non-linear constraints and objectives.

# The `@constraint` and `@objective` macros
# can be used for both levels to represent linear and quadratic
# constraints and objectives


# ## Quadratic constraints and objectives

using BilevelJuMP, Ipopt
model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-5))

@variable(Upper(model), x >= 2)
@variable(Lower(model), 3 <= y <= 5)

# We can add a non-linear objective to the upper level with `@NLobjective`

@NLobjective(Upper(model), Min, x^2 - y)

# We can also add a non-linear constraint to the upper level with `@NLconstraint`

@NLconstraint(Upper(model), x^2 + y^2 <= 100)

# `@NLobjective` is *not supported* in the lower level, but we can use
# `@constraint` to add a quadratic objective to the lower level.

@objective(Lower(model), Min, y^2)

#-

optimize!(model)

# All the quadratic objectives and constraints of the upper level can also
# be added with the `@constraint` and `@objective` macros. Hence, we can
# write the quivalent model:

using BilevelJuMP, Ipopt
model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-5))

@variable(Upper(model), x >= 2)
@variable(Lower(model), 3 <= y <= 5)

#-

@objective(Upper(model), Min, x^2 - y)

#-

@constraint(Upper(model), x^2 + y^2 <= 100)

#-

@objective(Lower(model), Min, y^2)

#-

optimize!(model)

# ## General non-linear constraints and objectives

# General non quadratic objectives and general non-linear constraints
# *can not* be added to the lower level, but they can be added to the upper level.

using BilevelJuMP, Ipopt
model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-5))

@variable(Upper(model), x >= 2)
@variable(Lower(model), 3 <= y <= 5)

#-

@NLobjective(Upper(model), Min, x^4 - sin(y))

#-

@NLconstraint(Upper(model), x^3 + y^3 <= 1000)

#-

@objective(Lower(model), Min, y^2)

#-

optimize!(model)

# !!! info
#     Conic constraints are supported in the lower level (see Conic Bilevel tutorial).