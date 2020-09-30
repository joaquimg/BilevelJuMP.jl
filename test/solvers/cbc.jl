using Cbc
using QuadraticToBinary

const CBC_OPTIMIZER = Cbc.Optimizer()
MOI.set(CBC_OPTIMIZER, MOI.Silent(), true)
const CBC_CACHE = MOIU.UniversalFallback(MOIU.Model{Float64}())
const CBC_CACHED = MOIU.CachingOptimizer(CBC_CACHE, CBC_OPTIMIZER)
const CBC_BRIDGED = MOI.Bridges.full_bridge_optimizer(CBC_CACHED, Float64)
MOI.Bridges.add_bridge(CBC_BRIDGED, MOI.Bridges.Constraint.SOCtoNonConvexQuadBridge{Float64})

push!(solvers, (opt = CBC_BRIDGED, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_sos, (opt = CBC_BRIDGED, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_indicator, (opt = CBC_BRIDGED, mode = BilevelJuMP.IndicatorMode()))

push!(solvers_fa, (opt = CBC_BRIDGED, mode = BilevelJuMP.FortunyAmatMcCarlMode(with_slack = false)))
push!(solvers_fa, (opt = CBC_BRIDGED, mode = BilevelJuMP.FortunyAmatMcCarlMode(with_slack = true)))
push!(solvers_fa2, (opt = CBC_BRIDGED, mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100)))

push!(solvers_bin_exp, (
    opt = QuadraticToBinary.Optimizer{Float64}(CBC_BRIDGED),
    mode = BilevelJuMP.StrongDualityInequalityMode(1e-9)))
MOI.set(solvers_bin_exp[end].opt, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

push!(solvers_bin_exp, (
    opt = QuadraticToBinary.Optimizer{Float64}(CBC_BRIDGED),
    mode = BilevelJuMP.StrongDualityEqualityMode()))
MOI.set(solvers_bin_exp[end].opt, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

#=
QTB_CBC_BRIDGED = MOI.Bridges.full_bridge_optimizer(
    QuadraticToBinary.Optimizer{Float64}(CBC_BRIDGED, lb=-1000,ub=1000),
    Float64)
MOI.Bridges.add_bridge(QTB_CBC_BRIDGED, MOI.Bridges.Constraint.SOCtoNonConvexQuadBridge{Float64})

push!(solvers_sos_quad_bin, (
    opt = QTB_CBC_BRIDGED,
    mode = BilevelJuMP.SOS1Mode()))
push!(solvers_fa_quad_bin, (
    opt = QTB_CBC_BRIDGED,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 1000, dual_big_M = 1000)))
=#
# push!(solvers_sos_quad, (
#     opt = QuadraticToBinary.Optimizer{Float64}(CBC_BRIDGED),
#     mode = BilevelJuMP.SOS1Mode()))
# MOI.set(solvers_bin_exp[end].opt, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)