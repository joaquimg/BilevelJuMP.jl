using Ipopt

IPO_OPT = Ipopt.Optimizer(print_level=0)
IPO = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(IPO_OPT)

push!(solvers_nlp_lowtol, (opt = IPO, mode = BilevelJuMP.ProductMode(1e-5)))
push!(solvers_nlp, (opt = IPO, mode = BilevelJuMP.ProductMode(1e-9)))
push!(solvers_nlp, (opt = IPO, mode = BilevelJuMP.ProductMode(1e-9, with_slack = true)))

push!(solvers, (opt = IPO, mode = BilevelJuMP.ProductMode(1e-9)))
push!(solvers_cached, (opt = IPO, mode = BilevelJuMP.ProductMode(1e-9)))
push!(solvers_quad, (opt = IPO, mode = BilevelJuMP.ProductMode(1e-9)))

push!(solvers_nlp_sd, (opt = IPO, mode = BilevelJuMP.StrongDualityMode(1e-9, inequality = true)))
push!(solvers_nlp_sd, (opt = IPO, mode = BilevelJuMP.StrongDualityMode(inequality = false)))
push!(solvers_nlp_sd_i, (opt = IPO, mode = BilevelJuMP.StrongDualityMode(1e-11, inequality = true)))
push!(solvers_nlp_sd_e, (opt = IPO, mode = BilevelJuMP.StrongDualityMode(inequality = false)))
