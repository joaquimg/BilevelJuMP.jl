# name = "BARON"
# add BARON

using BARON

BARON_OPT = BARON.Optimizer()
# MOI.set(BARON_OPT, MOI.Silent(), true)
# MOI.set(BARON_OPT, MOI.TimeLimitSec(), 5)
# BARON_OPT = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(BARON_OPT)

push!(solvers_nlp, (opt = BARON_OPT, mode = BilevelJuMP.ProductMode(1e-5)))