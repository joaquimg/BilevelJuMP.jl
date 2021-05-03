# name = "KNITRO"
# KNITRO = "67920dd8-b58e-52a8-8622-53c4cffbe346"

using KNITRO

KNITRO_OPT = KNITRO.Optimizer()
MOI.set(KNITRO_OPT, MOI.Silent(), true)
MOI.set(KNITRO_OPT, MOI.TimeLimitSec(), 5)
# KNITRO_OPT = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(KNITRO_OPT)

push!(solvers_complements, (opt = KNITRO_OPT, mode = BilevelJuMP.ComplementMode()))
