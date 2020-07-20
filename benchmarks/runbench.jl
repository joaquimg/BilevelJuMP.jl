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

with_att = JuMP.optimizer_with_attributes

SOLVERS = [
    (with_att(Gurobi.Optimizer, "TimeLimit" => 180), BilevelJuMP.SOS1Mode()),
    # (Xpress.Optimizer, BilevelJuMP.SOS1Mode()),
    # (Cbc.Optimizer, BilevelJuMP.SOS1Mode()),
    # (Ipopt.Optimizer, BilevelJuMP.ProductMode(1e-7)),
    # (GLPK.Optimizer, BilevelJuMP.SOS1Mode()),
    # (CPLEX.Optimizer, BilevelJuMP.SOS1Mode()),
    # (SCIP.Optimizer, BilevelJuMP.SOS1Mode()),
    # (Mosek.Optimizer, BilevelJuMP.SOS1Mode()),
    # (KNITRO.Optimizer, BilevelJuMP.SOS1Mode()),
]

# TODO
# Bonmin, Couenne, 

PROBLEMS = [
    # :SVR,
    # :RAND,
    # :TOLL,
    :FORECAST,
]

SEEDS = [
    1234,
    # 2345,
    # 3456,
    # 4567,
    # 5678,
    # 6789,
    # 7890,
    # 8901,
    # 9012,
    # 0123,
]

SVR = [
    # (features, sample_size)
    (  1,  10),
    (  2,  10),
    (  5,  10),
    (  1, 100),
    (  2, 100),
    (  5, 100),
    ( 10, 100),
    ( 20, 100),
    ( 50, 100),
    (  1,1000),
    (  2,1000),
    (  5,1000),
    ( 10,1000),
    ( 20,1000),
    ( 50,1000),
    (100,1000),
    (200,1000),
    (500,1000),
    # () for i in [1,2,5,10,20,50,100,200], j in [10, 100, 1000]
]

RAND = [
    # (rows, cols)
    (   5,   5),
    (  10,   5),
    (   5,  10),
    (  10,  10),
    (  50,  10),
    (  10,  50),
    (  50,  50),
    ( 100,  50),
    (  50, 100),
    ( 100, 100),
    ( 500, 100),
    ( 100, 500),
    ( 500, 500),
    (1000, 500),
    ( 500,1000),
    (1000,1000),
    (5000,1000),
    (1000,5000),
    (5000,5000),
]

TOLL = [
    # nodes
    5,
    10,
    20,
    50,
    100,
    200,
    500,
    1000,
    2000,
    5000,
]

FORECAST = [
    # (products, sample_size)
    (  1,  10),
    (  2,  10),
    (  5,  10),
    (  1, 100),
    (  2, 100),
    (  5, 100),
    ( 10, 100),
    ( 20, 100),
    ( 50, 100),
    (  1,1000),
    (  2,1000),
    (  5,1000),
    ( 10,1000),
    ( 20,1000),
    ( 50,1000),
    (100,1000),
    (200,1000),
    (500,1000),
    # () for i in [1,2,5,10,20,50,100,200], j in [10, 100, 1000]
]

for (optimizer, mode) in SOLVERS
    for seed in SEEDS
        if :SVR in PROBLEMS
            for (features, samples) in SVR
                @show features, samples, seed
                ret = bench_svr(features, samples, optimizer, mode, seed)
            end
        end
        if :RAND in PROBLEMS
            for (rows, cols) in RAND
                @show rows, cols, seed
                ret = bench_rand(rows, cols, 0.5, optimizer, mode, seed)
            end
        end
        if :TOLL in PROBLEMS
            for nodes in TOLL
                @show nodes, seed
                ret = bench_toll(nodes, optimizer, mode, seed)
            end
        end
        if :FORECAST in PROBLEMS
            for (products, samples) in FORECAST
                @show products, samples
                ret = bench_forecast(products, samples, optimizer, mode, seed)
            end
        end
    end
end