using SCIP
using QuadraticToBinary

SCIPopt = SCIP.Optimizer()
MOI.set(SCIPopt, MOI.Silent(), true)
MOI.set(SCIPopt, MOI.TimeLimitSec(), 5)
SCIPopt_CACHED = MOIU.CachingOptimizer(MOIU.UniversalFallback(MOIU.Model{Float64}()), SCIPopt)

push!(solvers, (opt = SCIPopt, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_sos, (opt = SCIPopt, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_unit, (opt = SCIPopt, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_indicator, (opt = SCIPopt, mode = BilevelJuMP.IndicatorMode()))

push!(solvers_fa, (opt = SCIPopt, mode = BilevelJuMP.FortunyAmatMcCarlMode(with_slack = false)))
push!(solvers_fa, (opt = SCIPopt, mode = BilevelJuMP.FortunyAmatMcCarlMode(with_slack = true)))
push!(solvers_fa2, (opt = SCIPopt, mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100)))

push!(solvers_cached, (opt = SCIPopt_CACHED, mode = BilevelJuMP.SOS1Mode()))

push!(solvers_bin_exp, (
    opt = QuadraticToBinary.Optimizer{Float64}(SCIPopt_CACHED),
    mode = BilevelJuMP.StrongDualityMode(1e-9, inequality = true)))
MOI.set(solvers_bin_exp[end].opt, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

push!(solvers_bin_exp, (
    opt = QuadraticToBinary.Optimizer{Float64}(SCIPopt_CACHED),
    mode = BilevelJuMP.StrongDualityMode(inequality = false)))
MOI.set(solvers_bin_exp[end].opt, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

QTB_SCIPopt_BRIDGED = MOI.Bridges.full_bridge_optimizer(
    QuadraticToBinary.Optimizer{Float64}(SCIPopt_CACHED, lb=-100,ub=100),
    Float64)
MOI.Bridges.add_bridge(QTB_SCIPopt_BRIDGED, MOI.Bridges.Constraint.SOCtoNonConvexQuadBridge{Float64})
MOI.set(QTB_SCIPopt_BRIDGED, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

push!(solvers_sos_quad_bin, (
    opt = QTB_SCIPopt_BRIDGED,
    mode = BilevelJuMP.SOS1Mode()))

push!(solvers_fa_quad_bin, (
    opt = QTB_SCIPopt_BRIDGED,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100)))

QTB_SCIPopt = QuadraticToBinary.Optimizer{Float64}(SCIPopt_CACHED, lb=-5, ub=5)
QTB_SCIPopt_B = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(QTB_SCIPopt)
push!(solvers_fa_quad_bin_mixed, (
    opt = QTB_SCIPopt_B,
    mode = BilevelJuMP.MixedMode(default=
        BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 10, dual_big_M = 10))))
MOI.set(QTB_SCIPopt_B, QuadraticToBinary.GlobalVariablePrecision(), 1e-3)


