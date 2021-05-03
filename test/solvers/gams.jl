# name = "GAMS"
# GAMS = "1ca51c6a-1b4d-4546-9ae1-53e0a243ab12"

using GAMS

GAMS_OPT = GAMS.Optimizer()
MOI.set(GAMS_OPT, MOI.Silent(), true)
MOI.set(GAMS_OPT, MOI.TimeLimitSec(), 5)
# GAMS_OPT = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(GAMS_OPT)

push!(solvers_complements, (opt = GAMS_OPT, mode = BilevelJuMP.ComplementMode()))
