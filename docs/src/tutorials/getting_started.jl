# # Getting started with BilevelJuMP

# This is a quick introduction to modeling and solving bilevel optimization
# with BilevelJuMP.

# If you are new to Julia, start with the
# [Getting started with Julia](https://jump.dev/JuMP.jl/stable/tutorials/getting_started/getting_started_with_julia/)
# from the JuMP documentation.

# If you are new to JuMP, start with the
# [Getting started with JuMP](https://jump.dev/JuMP.jl/stable/tutorials/getting_started/getting_started_with_JuMP/)
# from the JuMP documentation.

# ## Installation
#
# BilevelJuMP is a JuMP extension that be installed
# by using the built-in package manager.

# ```julia
# import Pkg
# Pkg.add("BilevelJuMP")
# ```

# That is all you need to model a bilevel optimization problem, but we want to
# also *solve* the problems. Therefore we need a solver, one such solver is
# `HiGHS.Optimizer`, which is provided by the
# [HiGHS.jl](https://github.com/jump-dev/HiGHS.jl) package.

# ```julia
# import Pkg
# Pkg.add("HiGHS")
# ```

# ## A first example

# We will solve the following bilevel optimization problem using BilevelJuMP
# and HiGHS. First we take a look in the entire code then we go through it
# step-by-step.

# Here is the example from Dempe (2002), Chapter 3.2, Page 25:

# ```math
# \begin{align*}
#     &\min_{x, y} && 3x + y \\
#     &\st && x \leq 5 \\
#     &    && y \leq 8 \\
#     &    && y \geq 0 \\
#     &    && x(y) \in
#      \begin{aligned}[t]
#         &\argmin_{x} && -x\\
#             &\st && x + y \leq 8\\
#             &    && 4x + y \geq 8\\
#             &    && 2x + y \leq 13\\
#             &    && 2x - 7y \leq 0
#      \end{aligned}
# \end{align*}
# ```

# Here is the complete code to model, solve and query results from the example:

using BilevelJuMP
using HiGHS

model = BilevelModel(
    HiGHS.Optimizer,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100))

@variable(Lower(model), x)

@variable(Upper(model), y)

@objective(Upper(model), Min, 3x + y)

@constraint(Upper(model), u1, x <= 5)
@constraint(Upper(model), u2, y <= 8)
@constraint(Upper(model), u3, y >= 0)

@objective(Lower(model), Min, -x)

@constraints(Lower(model), l1, x +  y <= 8)
@constraints(Lower(model), l2, 4x +  y >= 8)
@constraints(Lower(model), l3, 2x +  y <= 13)
@constraints(Lower(model), l4, 2x - 7y <= 0)

print(model)

optimize!(model)

termination_status(model)

primal_status(model)

objective_value(model)

value(x)

value(y)

dual(l1)

dual(l2)

# ## Step-by-step

# Once installed, BilevelJuMP can be loaded into julia:

using BilevelJuMP

# Note that JuMP comes inside BilevelJuMP, and does not need to be installed
# separately. Once loaded, all JuMP functions are exported along with the 
# BilevelJuMP additional functions.

# We include a solver, in this case HiGHS:

using HiGHS

# Just like regular JuMP has the `Model` function to initialize an optimization
# problem, BilevelJuMP has the `BilevelModel` function that takes a solver as
# a first positional argument, in this case `HiGHS.Optimizer` and a `mode`
# keyword argument that selects
# a bilevel solution method, in this case, `BilevelJuMP.FortunyAmatMcCarlMode`.
# Note that `BilevelJuMP.FortunyAmatMcCarlMode` takes two optional keyword
# arguments: `primal_big_M` and `dual_big_M` which have to be larger than the
# value of all primal and dual variavle sof the lower level respectively to
# guarantee that the solution is no eliminated.

model = BilevelModel(
    HiGHS.Optimizer,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100))

# For more on `mode`s and solutions methods, see XXX.

# We can proceed, as usual in JuMP models and incrementally build our bilevel
# problem. We use the same macros as JuMP.

# Variables are modeled using the `@variable` macro, but in bilevel problems
# we must define which level the variable is decided, then we use the
# `Upper` and `Lower` constructors to direct variable to the proper levels:

@variable(Lower(model), x)

@variable(Upper(model), y)

# The same goes for objective that are modeled with the `@objective` macro:

@objective(Upper(model), Min, 3x + y)

# and constraints that are modeled with the `@objective` macro:

@constraint(Upper(model), u1, x <= 5)
@constraint(Upper(model), u2, y <= 8)
@constraint(Upper(model), u3, y >= 0)

# repeat for the lower level:

@objective(Lower(model), Min, -x)

@constraints(Lower(model), l1, x +  y <= 8)
@constraints(Lower(model), l2, 4x +  y >= 8)
@constraints(Lower(model), l3, 2x +  y <= 13)
@constraints(Lower(model), l4, 2x - 7y <= 0)