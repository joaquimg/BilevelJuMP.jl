# name = "Alpine"
# Alpine = "07493b3f-dabb-5b16-a503-4139292d7dd4"

using Alpine, Ipopt, Cbc

const ALP_IPOPT = optimizer_with_attributes(Ipopt.Optimizer, MOI.Silent() => true, "sb" => "yes")
const ALP_CBC = optimizer_with_attributes(Cbc.Optimizer, MOI.Silent() => true)

# ALP_OPT = optimizer_with_attributes(Alpine.Optimizer, "nlp_solver" => ALP_IPOPT,
# "mip_solver" => ALP_CBC,
# "loglevel" => 100)
# ALP = ALP_OPT

ALP_OPT = Alpine.Optimizer()
MOI.set(ALP_OPT, MOI.RawParameter("nlp_solver"), ALP_IPOPT)
MOI.set(ALP_OPT, MOI.RawParameter("mip_solver"), ALP_CBC)
ALP = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(ALP_OPT)

push!(solvers_nlp_lowtol, (opt = ALP, mode = BilevelJuMP.ProductMode(1e-5)))
push!(solvers_nlp, (opt = ALP, mode = BilevelJuMP.ProductMode(1e-9)))
push!(solvers_nlp, (opt = ALP, mode = BilevelJuMP.ProductMode(1e-9, with_slack = true)))

push!(solvers, (opt = ALP, mode = BilevelJuMP.ProductMode(1e-9)))
push!(solvers_cached, (opt = ALP, mode = BilevelJuMP.ProductMode(1e-9)))
push!(solvers_quad, (opt = ALP, mode = BilevelJuMP.ProductMode(1e-9)))

push!(solvers_nlp_sd, (opt = ALP, mode = BilevelJuMP.StrongDualityMode(1e-9, inequality = true)))
push!(solvers_nlp_sd, (opt = ALP, mode = BilevelJuMP.StrongDualityMode(inequality = false)))
push!(solvers_nlp_sd_i, (opt = ALP, mode = BilevelJuMP.StrongDualityMode(1e-9, inequality = true)))
push!(solvers_nlp_sd_e, (opt = ALP, mode = BilevelJuMP.StrongDualityMode(inequality = false)))
