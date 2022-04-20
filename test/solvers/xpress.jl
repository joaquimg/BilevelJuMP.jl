using Pkg
Pkg.add(name="Xpress")#, version="0.13.2")
Pkg.build("Xpress")
using Xpress
using QuadraticToBinary

const XPRESS = MOI.instantiate(Xpress.Optimizer, with_bridge_type = Float64)
MOI.set(XPRESS, MOI.Silent(), true)

push!(solvers, (opt = XPRESS, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_sos, (opt = XPRESS, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_quad, (opt = XPRESS, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_sos_quad, (opt = XPRESS, mode = BilevelJuMP.SOS1Mode()))

push!(solvers_indicator, (opt = XPRESS, mode = BilevelJuMP.IndicatorMode()))

push!(solvers_fa, (opt = XPRESS, mode = BilevelJuMP.FortunyAmatMcCarlMode(with_slack = false)))
push!(solvers_fa, (opt = XPRESS, mode = BilevelJuMP.FortunyAmatMcCarlMode(with_slack = true)))
push!(solvers_fa2, (opt = XPRESS, mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100)))
push!(solvers_cached, (opt = XPRESS, mode = BilevelJuMP.SOS1Mode()))

push!(solvers_bin_exp, (
    opt = QuadraticToBinary.Optimizer{Float64}(XPRESS),
    mode = BilevelJuMP.StrongDualityMode(1e-9, inequality = true)))
MOI.set(solvers_bin_exp[end].opt, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

push!(solvers_bin_exp, (
    opt = QuadraticToBinary.Optimizer{Float64}(XPRESS),
    mode = BilevelJuMP.StrongDualityMode(inequality = false)))
MOI.set(solvers_bin_exp[end].opt, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

QTB_XPRESS_BRIDGED = MOI.Bridges.full_bridge_optimizer(
    QuadraticToBinary.Optimizer{Float64}(XPRESS, lb=-100,ub=100),
    Float64)
MOI.Bridges.add_bridge(QTB_XPRESS_BRIDGED, MOI.Bridges.Constraint.SOCtoNonConvexQuadBridge{Float64})
MOI.set(QTB_XPRESS_BRIDGED, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

push!(solvers_sos_quad_bin, (
    opt = QTB_XPRESS_BRIDGED,
    mode = BilevelJuMP.SOS1Mode()))

push!(solvers_fa_quad_bin, (
    opt = QTB_XPRESS_BRIDGED,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100)))

QTB_XPRESS = QuadraticToBinary.Optimizer{Float64}(XPRESS, lb=-5, ub=5)
QTB_XPRESS_B = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(QTB_XPRESS)
MOI.set(QTB_XPRESS_B, QuadraticToBinary.GlobalVariablePrecision(), 1e-3)
push!(solvers_fa_quad_bin_mixed, (
    opt = QTB_XPRESS_B,
    mode = BilevelJuMP.MixedMode(default=
        BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 10, dual_big_M = 10))))