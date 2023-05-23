# # Using QuadraticToBinary

# [QuadraticToBinary.jl](https://github.com/joaquimg/QuadraticToBinary.jl) is a
# package that converts quadratic terms in constraints and objective. To do so
# the pack acts like a solver on top of the real solver and most data is forwarded
# directly to the solver itself. For many solvers it is enough to use:

using BilevelJuMP, QuadraticToBinary, HiGHS

SOLVER = HiGHS.Optimizer()
Q_SOLVER = QuadraticToBinary.Optimizer{Float64}(SOLVER, lb = -10, ub = 10)
model = BilevelModel(()->Q_SOLVER, mode = BilevelJuMP.ProductMode(1e-6))

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
     x +  y <= 8
    4x +  y >= 8
    2x +  y <= 13
    2x - 7y <= 0
end)

optimize!(model)

objective_value(model)
@assert abs(objective_value(model) - (3 * (3.5 * 8/15) + 8/15)) < 1e-1 # src


# However, this might lead to some solver not supporting certain functionality like SCIP.
# In this case we need to:

using SCIP

# !!! warning
#     SCIP requires a non-standard installation procedure in windows.
#     See [SCIP.jl](https://github.com/scipopt/SCIP.jl#custom-installations) for
#     more details.

SOLVER = SCIP.Optimizer()

CACHED_SOLVER = MOI.Utilities.CachingOptimizer(
    MOI.Utilities.UniversalFallback(MOI.Utilities.Model{Float64}()), SOLVER)

Q_SOLVER = QuadraticToBinary.Optimizer{Float64}(CACHED_SOLVER)

BilevelModel(()->Q_SOLVER, mode = BilevelJuMP.ProductMode(1e-5))

# Note that we used `()->Q_SOLVER` instead of just `Q_SOLVER` because `BilevelModel`
# requires as constructor and not an instance of an object.
