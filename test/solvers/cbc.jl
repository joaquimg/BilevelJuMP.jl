using Cbc

const CBC_OPTIMIZER = Cbc.Optimizer()
MOI.set(CBC_OPTIMIZER, MOI.Silent(), true)
const CBC_CACHE = MOIU.UniversalFallback(MOIU.Model{Float64}())
const CBC_CACHED = MOIU.CachingOptimizer(CBC_CACHE, CBC_OPTIMIZER)
const CBC_BRIDGED = MOI.Bridges.full_bridge_optimizer(CBC_CACHED, Float64)

push!(solvers, (opt = CBC_BRIDGED, mode = BilevelJuMP.SOS1Mode()))