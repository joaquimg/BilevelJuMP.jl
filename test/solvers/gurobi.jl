using Gurobi

const GRB_OPTIMIZER = Gurobi.Optimizer(OutputFlag=0)
MOI.set(GRB_OPTIMIZER, MOI.Silent(), true)
const GRB_CACHE = MOIU.UniversalFallback(MOIU.Model{Float64}())
const GRB_CACHED = MOIU.CachingOptimizer(GRB_CACHE, GRB_OPTIMIZER)
const GRB_BRIDGED = MOI.Bridges.full_bridge_optimizer(GRB_CACHED, Float64)

push!(solvers, (opt = GRB_BRIDGED, mode = BilevelJuMP.SOS1Mode))
push!(solvers_quad, (opt = GRB_BRIDGED, mode = BilevelJuMP.SOS1Mode))