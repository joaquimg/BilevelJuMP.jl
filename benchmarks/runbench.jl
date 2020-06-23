using Random
using JuMP, BilevelJuMP, Cbc, Gurobi

include("svr.jl")
include("rand.jl")
include("forecast.jl")
include("toll.jl")

CBC_OPTIMIZER = Gurobi.Optimizer()
MOI.set(CBC_OPTIMIZER, MOI.Silent(), true)
CBC_CACHE = MOIU.UniversalFallback(MOIU.Model{Float64}())
CBC_CACHED = MOIU.CachingOptimizer(CBC_CACHE, CBC_OPTIMIZER)
CBC_BRIDGED = MOI.Bridges.full_bridge_optimizer(CBC_CACHED, Float64)


mode = BilevelJuMP.SOS1Mode()
optimizer = CBC_BRIDGED
bench_svr(2, 20, optimizer, mode)

bench_rand(5, 5, 0.5, optimizer, mode)

bench_toll(5, optimizer, mode)
bench_forecast(2, 5, optimizer, mode)