using Xpress

const XPR_OPTIMIZER = Xpress.Optimizer()
if !Sys.iswindows()
    MOI.set(XPR_OPTIMIZER, MOI.Silent(), true)
end
const XPR_CACHE = MOIU.UniversalFallback(MOIU.Model{Float64}())
const XPR_CACHED = MOIU.CachingOptimizer(XPR_CACHE, XPR_OPTIMIZER)
const XPR_BRIDGED = MOI.Bridges.full_bridge_optimizer(XPR_CACHED, Float64)

push!(solvers, (opt = XPR_BRIDGED, mode = BilevelJuMP.PositiveSOS1Mode()))
push!(solvers_sos, (opt = XPR_BRIDGED, mode = BilevelJuMP.PositiveSOS1Mode()))
push!(solvers_quad, (opt = XPR_BRIDGED, mode = BilevelJuMP.PositiveSOS1Mode()))
push!(solvers_sos_quad, (opt = XPR_BRIDGED, mode = BilevelJuMP.PositiveSOS1Mode()))