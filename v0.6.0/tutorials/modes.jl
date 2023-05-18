# # Modes overview

# There are several ways to solve bilevel problems with BilevelJuMP. The main
# difference between them is the way the complementarity constraints are
# reformulated. The reformulation method is set with the `mode` option of the
# `BilevelModel` constructor.
# 
# The `mode`s available are:
# 
# - `BilevelJuMP.SOS1Mode()`: uses SOS1 constraints to model complementarity
#   constraints and solve the problem with MIP solvers (Cbc, Xpress, Gurobi,
#   CPLEX, SCIP).
# 
# - `BilevelJuMP.IndicatorMode()`: uses Indicator constraints to model
#     complementarity constraints and solve the problem with MIP solvers (Cbc,
#     Xpress, Gurobi, CPLEX, SCIP).
# 
# - `BilevelJuMP.BigMMode()`: uses the Fortuny-Amat and McCarl
#     reformulation that requires a MIP solver with very basic functionality,
#     i.e., just binary variables are needed. The main drawback of this method is
#     that one must provide bounds for all primal and dual variables. However, if
#     good bounds are provided, this method can be more efficient than the
#     previous. Bound hints to compute the big-Ms can be passed with the methods:
#     `set_primal_(upper\lower)_bound_hint(variable, bound)`, for primals; and
#     `set_dual_(upper\lower)_bound_hint(constraint, bound)` for duals. We can
#     also call `FortunyAmatMcCarlMode(primal_big_M = vp, dual_big_M = vd)`,
#     where `vp` and `vd` are, respectively, the big M fallback values for
#     primal and dual variables, these are used when some variables have no given
#     bounds, otherwise the given bounds are used instead.
# 
# - `BilevelJuMP.ProductMode()`: reformulates the complementarity constraints as
#     products so that the problem can be solved by NLP (Ipopt, KNITRO) solvers or
#     even MIP solvers with the aid of binary expansions
#     (see `QuadraticToBinary.jl`). Note that binary expansions require variables
#     to have upper and lower bounds. Also, note that the `Gurobi` solver supports
#     products, but requires
#     [setting the `"NonConvex"` options](https://github.com/jump-dev/Gurobi.jl#using-gurobi-v90-and-you-got-an-error-like-q-not-psd).
# 
# - `BilevelJuMP.MixedMode(default = mode)`: where `mode` is one of the other
#     modes described above. With this method it is possible to set complementarity
#     reformulations per constraint with `BilevelJuMP.set_mode(constraint, mode)`,
#     where `constraint` is a constraint of the bilevel problem and `mode` is one
#     of the modes described above. If no mode is set for a constraint, the
#     `default` mode is used instead. To set a mode to reformulate variables
#     bounds use: `BilevelJuMP.set_mode(variable, mode)`.
# 
# - `BilevelJuMP.StrongDualityMode`: this mode is not a complementarity
#     reformulation method, instead, all complementarity constraints are replaced
#     by constraints enforcing the strong duality.
#     This mode is especially amenable for NLP solvers (Ipopt, KNITRO).
#     MIP solvers can also be used but they will have to deal with the products
#     of variables crated or the will have to be used in conjunction with
#     `QuadraticToBinary.jl`.
# 
# 
# The following example shows how to solve a classic bilevel problem.

# We start loading all libraries needed for this example.

using BilevelJuMP, HiGHS, Ipopt, SCIP

# Now we create a bilevel model with the `BilevelModel` constructor with no
# solver nor mode specified.

model = BilevelModel()

@variable(Lower(model), x)
@variable(Upper(model), y)

@objective(Upper(model), Min, 3x + y)
@constraints(Upper(model), begin
    x <= 5
    y <= 8
    y >= 0
end)

@objective(Lower(model), Min, -x)
@constraints(Lower(model), begin
    c1,  x +  y <= 8
    c2, 4x +  y >= 8
    c3, 2x +  y <= 13
    c4, 2x - 7y <= 0
end)

# ## `BigMMode` and `HiGHS.Optimizer`

set_optimizer(model, HiGHS.Optimizer)

BilevelJuMP.set_mode(model,
    BilevelJuMP.BigMMode(primal_big_M = 100, dual_big_M = 100))

optimize!(model)

objective_value(model)
@assert abs(objective_value(model) - (3 * (7/2 * 8/15) + 8/15)) < 1e-1 # src

# ## `SOS1Mode` and `SCIP.Optimizer`

set_optimizer(model, SCIP.Optimizer)

BilevelJuMP.set_mode(model, BilevelJuMP.SOS1Mode())

optimize!(model)

objective_value(model)

@assert abs(objective_value(model) - (3 * (3.5 * 8/15) + 8/15)) < 1e-1 # src

# !!! warning
#     SCIP requires a non-standard installation procedure in windows.
#     See [SCIP.jl](https://github.com/scipopt/SCIP.jl#custom-installations) for
#     more details.

# ## `IndicatorMode` and `SCIP.Optimizer`

set_optimizer(model, SCIP.Optimizer)

BilevelJuMP.set_mode(model, BilevelJuMP.IndicatorMode())

optimize!(model)

objective_value(model)

@assert abs(objective_value(model) - (3 * (3.5 * 8/15) + 8/15)) < 1e-1 # src

# !!! warning
#     SCIP requires a non-standard installation procedure in windows.
#     See [SCIP.jl](https://github.com/scipopt/SCIP.jl#custom-installations) for
#     more details.

# ## `ProductMode` and `Ipopt.Optimizer`

set_optimizer(model, Ipopt.Optimizer)

BilevelJuMP.set_mode(model, BilevelJuMP.ProductMode())

optimize!(model)

objective_value(model)

@assert abs(objective_value(model) - (3 * (7/2 * 8/15) + 8/15)) < 1e-1 # src

# ## `StrongDualityMode` and `Ipopt.Optimizer`

set_optimizer(model, Ipopt.Optimizer)

BilevelJuMP.set_mode(model, BilevelJuMP.StrongDualityMode())

optimize!(model)

objective_value(model)

@assert abs(objective_value(model) - (3 * (7/2 * 8/15) + 8/15)) < 1e-1 # src

# ## `MixedMode` and `SCIP.Optimizer`

set_optimizer(model, SCIP.Optimizer)

BilevelJuMP.set_mode(model, BilevelJuMP.MixedMode(default = BilevelJuMP.SOS1Mode()))

BilevelJuMP.set_mode(c1, BilevelJuMP.IndicatorMode())

BilevelJuMP.set_mode(c3, BilevelJuMP.SOS1Mode())

optimize!(model)

objective_value(model)

@assert abs(objective_value(model) - (3 * (3.5 * 8/15) + 8/15)) < 1e-1 # src

# !!! warning
#     SCIP requires a non-standard installation procedure in windows.
#     See [SCIP.jl](https://github.com/scipopt/SCIP.jl#custom-installations) for
#     more details.
