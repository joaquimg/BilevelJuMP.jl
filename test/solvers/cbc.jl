using Cbc
using QuadraticToBinary

CBC_OPTIMIZER = Cbc.Optimizer()
# MOI.set(CBC_OPTIMIZER, MOI.Silent(), false)
MOI.set(CBC_OPTIMIZER, MOI.TimeLimitSec(), 5)

CBC = CBC_OPTIMIZER

push!(solvers, (opt = CBC, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_sos, (opt = CBC, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_indicator, (opt = CBC, mode = BilevelJuMP.IndicatorMode()))

push!(solvers_fa, (opt = CBC, mode = BilevelJuMP.FortunyAmatMcCarlMode(with_slack = false)))
push!(solvers_fa, (opt = CBC, mode = BilevelJuMP.FortunyAmatMcCarlMode(with_slack = true)))
push!(solvers_fa2, (opt = CBC, mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100)))

CBC_CACHED = MOIU.CachingOptimizer(MOIU.UniversalFallback(MOIU.Model{Float64}()), CBC_OPTIMIZER)
push!(solvers_cached, (opt = CBC_CACHED, mode = BilevelJuMP.SOS1Mode()))

push!(solvers_bin_exp, (
    opt = QuadraticToBinary.Optimizer{Float64}(CBC_CACHED),
    mode = BilevelJuMP.StrongDualityMode(1e-9, inequality = true)))
MOI.set(solvers_bin_exp[end].opt, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

push!(solvers_bin_exp, (
    opt = QuadraticToBinary.Optimizer{Float64}(CBC_CACHED),
    mode = BilevelJuMP.StrongDualityMode(inequality = false)))
MOI.set(solvers_bin_exp[end].opt, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

QTB_CBC_BRIDGED = MOI.Bridges.full_bridge_optimizer(
    QuadraticToBinary.Optimizer{Float64}(CBC_CACHED, lb=-100,ub=100),
    Float64)
MOI.Bridges.add_bridge(QTB_CBC_BRIDGED, MOI.Bridges.Constraint.SOCtoNonConvexQuadBridge{Float64})
MOI.set(QTB_CBC_BRIDGED, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

push!(solvers_sos_quad_bin, (
    opt = QTB_CBC_BRIDGED,
    mode = BilevelJuMP.SOS1Mode()))

push!(solvers_fa_quad_bin, (
    opt = QTB_CBC_BRIDGED,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100)))

QTB_CBC = QuadraticToBinary.Optimizer{Float64}(CBC_CACHED, lb=-5, ub=5)
QTB_CBC_B = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(QTB_CBC)
push!(solvers_fa_quad_bin_mixed, (
    opt = QTB_CBC_B,
    mode = BilevelJuMP.MixedMode(default=
        BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 10, dual_big_M = 10))))
MOI.set(QTB_CBC_B, QuadraticToBinary.GlobalVariablePrecision(), 1e-3)


