# # Variable levels

# In a bilevel problem every variable belongs to a level: it is decided either
# by the upper level (the leader) or by the lower level (the follower).
# `Upper` and `Lower` say which level decides a variable, and both make the
# variable visible to the other level. `UpperOnly` and `LowerOnly` restrict that
# visibility. In some situations this might simplify formulations.

# This tutorial goes through the four constructors and when each is the right
# choice.

using BilevelJuMP, HiGHS

# ## `Upper` and `Lower`

# These are the two used in most models. A variable created with either is
# *shared*: both levels may use it in their objectives and constraints.

model = BilevelModel(
    HiGHS.Optimizer,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100))

@variable(Upper(model), 0 <= q <= 10)

@variable(Lower(model), 0 <= r <= 10)

# `r` is decided by the lower level, but the upper level is still free to use it:

@objective(Upper(model), Min, q - 2r)

# This sharing is what makes the problem bilevel. The leader optimizes over the
# follower's response, so it has to be able to refer to the variables the
# follower decides. Likewise the follower takes `q` as given, a parameter of its
# own problem:

@objective(Lower(model), Min, r)

@constraint(Lower(model), q + r <= 8)

optimize!(model)

objective_value(model)

# ## `UpperOnly`

# `UpperOnly` creates a variable that only the upper level knows about. The
# lower level cannot use it.

# This changes the problem that is solved, rather than only stating intent. A
# variable created with `Upper` enters the lower level problem as a parameter,
# so it appears in the lower level optimality conditions that the reformulation
# builds. An `UpperOnly` variable does not appear there at all.

# Use it for leader decisions the follower must not react to:

model = BilevelModel(
    HiGHS.Optimizer,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100))

@variable(UpperOnly(model), 0 <= p <= 10)

@variable(Upper(model), 0 <= q <= 10)

@variable(Lower(model), 0 <= r <= 10)

@objective(Upper(model), Min, p + q - 2r)

@constraint(Upper(model), p + q >= 3)

@objective(Lower(model), Min, r)

@constraint(Lower(model), q + r <= 8)

optimize!(model)

value.([p, q, r])

# ## `LowerOnly`

# `LowerOnly` creates a variable that only the lower level knows about. The
# upper level cannot use it, in its objective or in its constraints.

# Use it for quantities that are internal to the follower, such as auxiliary or
# slack variables that only exist to express the lower level problem. Declaring
# them with `Lower` would also work, but `LowerOnly` states the intent and turns
# an accidental use in the upper level into an error.

model = BilevelModel(
    HiGHS.Optimizer,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100))

@variable(Upper(model), 0 <= q <= 10)

@variable(Lower(model), 0 <= r <= 10)

@variable(LowerOnly(model), s >= 1)

@objective(Upper(model), Min, q - 2r)

@objective(Lower(model), Min, r + s)

@constraint(Lower(model), q + r + s <= 8)

optimize!(model)

value.([q, r, s])

# ## Levels are enforced

# Using a variable in a level that cannot see it is an error, not a silent
# reinterpretation. With the model above, `s` is known only to the lower level,
# so putting it in the upper objective fails:

# ```julia
# @objective(Upper(model), Min, s)
# # ERROR: Variable s belonging Only to LOWER_ONLY level, was added in the
# # UPPER_ONLY level.
# ```

# The same holds the other way around for an `UpperOnly` variable used in a
# lower level constraint.

# !!! info
#     `UpperOnly` and `LowerOnly` apply to variables only. Objectives and
#     constraints are always added with `Upper` and `Lower`, which decide the
#     level they belong to.

# ## Querying variables per level

# Whatever its level, every variable is part of the single `BilevelModel`, so
# `value` works for all of them after a solve, and `all_variables` lists them
# all:

all_variables(model)

# The level specific queries list only the variables that level can see, which
# is a quick way to check a model is wired the way it was meant to be:

all_variables(Upper(model))

# Note that `s` is absent above, and present below:

all_variables(Lower(model))

# ## Summary

# | constructor | decided by | visible to upper | visible to lower |
# |:---|:---|:---|:---|
# | `Upper(model)` | upper | yes | yes |
# | `Lower(model)` | lower | yes | yes |
# | `UpperOnly(model)` | upper | yes | no |
# | `LowerOnly(model)` | lower | no | yes |

# There is a fifth case, `DualOf`, for upper level variables that are the dual
# of a lower level constraint. See
# [Dual variables of the lower level](@ref) for that one.
