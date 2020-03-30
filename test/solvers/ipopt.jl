using Ipopt

const IPO_OPTIMIZER = Ipopt.Optimizer(print_level=0)
MOI.set(IPO_OPTIMIZER, MOI.Silent(), true)
const IPO_CACHE = MOIU.UniversalFallback(MOIU.Model{Float64}())
const IPO_CACHED = MOIU.CachingOptimizer(IPO_CACHE, IPO_OPTIMIZER)
const IPO_BRIDGED = MOI.Bridges.full_bridge_optimizer(IPO_CACHED, Float64)
# const IPO_BRIDGED = MOI.Bridges.full_bridge_optimizer(IPO_OPTIMIZER, Float64)

push!(solvers_nlp, (opt = IPO_BRIDGED, mode = BilevelJuMP.ProductMode{Float64}(1e-9)))
push!(solvers_nlp, (opt = IPO_BRIDGED, mode = BilevelJuMP.ProductWithSlackMode{Float64}(1e-9)))

push!(solvers, (opt = IPO_BRIDGED, mode = BilevelJuMP.ProductMode{Float64}(1e-9)))
push!(solvers_quad, (opt = IPO_BRIDGED, mode = BilevelJuMP.ProductMode{Float64}(1e-9)))