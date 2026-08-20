# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

# Load the solver from Xpress_jll rather than a local installation, so no
# `XPRESSDIR` / `Pkg.build("Xpress")` is needed. This matches the approach in
# Xpress.jl's own test suite. Requires Xpress.jl >= v0.18.
import Xpress_jll
ENV["XPRESS_JL_LIBRARY"] = Xpress_jll.libxprs
# CI writes the licence and exports XPAUTH_PATH; otherwise fall back to the
# raw licence in XPAUTH_XPR, or to whatever the local install already uses.
if !haskey(ENV, "XPAUTH_PATH") && haskey(ENV, "XPAUTH_XPR")
    xpauth_xpr = joinpath(@__DIR__, "xpauth.xpr")
    write(xpauth_xpr, ENV["XPAUTH_XPR"])
    ENV["XPAUTH_PATH"] = xpauth_xpr
end
# Initialize explicitly rather than relying on Xpress.jl's automatic init,
# which does not pick up the licence when the library comes from Xpress_jll
# (XPRSinit fails silently, and the first XPRSprob call then reports
# "global environment not initialised"). This mirrors Xpress.jl's own tests.
ENV["XPRESS_JL_NO_AUTO_INIT"] = "true"
using Xpress
Xpress.initialize(; verbose = false, xpauth_path = ENV["XPAUTH_PATH"])
using QuadraticToBinary

const XPRESS = MOI.instantiate(Xpress.Optimizer; with_bridge_type = Float64)
MOI.set(XPRESS, MOI.Silent(), true)

# Xpress 9.8 mishandles the SOS1 constraints that SOS1Mode uses to encode
# complementarity: it reports OPTIMAL while its own log admits
#   Max integer violation (abs) : 1.000e+00
# and returns points that are jointly feasible but not lower-level optimal
# (e.g. jump_06 gives x=3, y=6, obj=-21 where the follower's optimum at
# x=3 is y=2.5, so the true answer is x=4, y=4, obj=-12). ProductMode and
# FortunyAmatMcCarlMode on the same models are correct and report no integer
# violation, so this is specific to SOS1. Disabled until resolved; see #245.
# push!(solvers, (opt = XPRESS, mode = BilevelJuMP.SOS1Mode()))
# push!(solvers_sos, (opt = XPRESS, mode = BilevelJuMP.SOS1Mode()))
# push!(solvers_quad, (opt = XPRESS, mode = BilevelJuMP.SOS1Mode()))
# push!(solvers_sos_quad, (opt = XPRESS, mode = BilevelJuMP.SOS1Mode()))

push!(solvers_indicator, (opt = XPRESS, mode = BilevelJuMP.IndicatorMode()))

push!(
    solvers_fa,
    (
        opt = XPRESS,
        mode = BilevelJuMP.FortunyAmatMcCarlMode(; with_slack = false),
    ),
)
push!(
    solvers_fa,
    (
        opt = XPRESS,
        mode = BilevelJuMP.FortunyAmatMcCarlMode(; with_slack = true),
    ),
)
push!(
    solvers_fa2,
    (
        opt = XPRESS,
        mode = BilevelJuMP.FortunyAmatMcCarlMode(;
            primal_big_M = 100,
            dual_big_M = 100,
        ),
    ),
)
# SOS1Mode disabled for Xpress, see the note above.
# push!(solvers_cached, (opt = XPRESS, mode = BilevelJuMP.SOS1Mode()))

push!(
    solvers_bin_exp,
    (
        opt = QuadraticToBinary.Optimizer{Float64}(XPRESS),
        mode = BilevelJuMP.StrongDualityMode(1e-9; inequality = true),
    ),
)
MOI.set(
    solvers_bin_exp[end].opt,
    QuadraticToBinary.GlobalVariablePrecision(),
    1e-5,
)

push!(
    solvers_bin_exp,
    (
        opt = QuadraticToBinary.Optimizer{Float64}(XPRESS),
        mode = BilevelJuMP.StrongDualityMode(; inequality = false),
    ),
)
MOI.set(
    solvers_bin_exp[end].opt,
    QuadraticToBinary.GlobalVariablePrecision(),
    1e-5,
)

QTB_XPRESS_BRIDGED = MOI.Bridges.full_bridge_optimizer(
    QuadraticToBinary.Optimizer{Float64}(XPRESS; lb = -100, ub = 100),
    Float64,
)
MOI.Bridges.add_bridge(
    QTB_XPRESS_BRIDGED,
    MOI.Bridges.Constraint.SOCtoNonConvexQuadBridge{Float64},
)
MOI.set(QTB_XPRESS_BRIDGED, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

# SOS1Mode disabled for Xpress, see the note above.
# push!(
#     solvers_sos_quad_bin,
#     (opt = QTB_XPRESS_BRIDGED, mode = BilevelJuMP.SOS1Mode()),
# )

push!(
    solvers_fa_quad_bin,
    (
        opt = QTB_XPRESS_BRIDGED,
        mode = BilevelJuMP.FortunyAmatMcCarlMode(;
            primal_big_M = 100,
            dual_big_M = 100,
        ),
    ),
)

QTB_XPRESS = QuadraticToBinary.Optimizer{Float64}(XPRESS; lb = -5, ub = 5)
QTB_XPRESS_B = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(QTB_XPRESS)
# 1e-3 leaves the binary expansion too coarse for the conic tests: the SOC
# constraint in jump_conic04 ends up violated by ~7e-3 against a 1e-3
# tolerance. Tighten it as the bridged SCIP optimizer does.
MOI.set(QTB_XPRESS_B, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)
push!(
    solvers_fa_quad_bin_mixed,
    (
        opt = QTB_XPRESS_B,
        mode = BilevelJuMP.MixedMode(;
            default = BilevelJuMP.FortunyAmatMcCarlMode(;
                primal_big_M = 10,
                dual_big_M = 10,
            ),
        ),
    ),
)
