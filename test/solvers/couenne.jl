# name = "Couenne_jll"
# Couenne_jll = "f09e9e23-9010-5c9e-b679-9f1d8f79b85c"
# name = "AmplNLWriter"
# AmplNLWriter = "7c4d4715-977e-5154-bfe0-e096adeac482"

using AmplNLWriter, Couenne_jll

COUENNE_OPT = AmplNLWriter.Optimizer(Couenne_jll.amplexe)
# MOI.set(COUENNE_OPT, MOI.Silent(), true)
# MOI.set(COUENNE_OPT, MOI.TimeLimitSec(), 5)
# COUENNE_OPT = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(COUENNE_OPT)

push!(solvers_nlp, (opt = COUENNE_OPT, mode = BilevelJuMP.ProductMode(1e-5)))