# Gurobi = "2e9cd046-0924-5485-92f1-d5272153d98b"

using Gurobi
using QuadraticToBinary

const GRB_OPTIMIZER = Gurobi.Optimizer()
MOI.set(GRB_OPTIMIZER, MOI.Silent(), true)
const GRB_CACHE = MOIU.UniversalFallback(MOIU.Model{Float64}())
const GRB_CACHED = MOIU.CachingOptimizer(GRB_CACHE, GRB_OPTIMIZER)
const GRB_BRIDGED = MOI.Bridges.full_bridge_optimizer(GRB_CACHED, Float64)

push!(solvers, (opt = GRB_BRIDGED, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_sos, (opt = GRB_BRIDGED, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_quad, (opt = GRB_BRIDGED, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_sos_quad, (opt = GRB_BRIDGED, mode = BilevelJuMP.SOS1Mode()))

push!(solvers_fa2, (opt = GRB_BRIDGED, mode = BilevelJuMP.FortunyAmatMcCarlMode(with_slack = false, primal_big_M = 100, dual_big_M = 100)))


push!(solvers_sos_quad_bin, (
    opt = QuadraticToBinary.Optimizer{Float64}(GRB_BRIDGED),
    mode = BilevelJuMP.SOS1Mode()))
