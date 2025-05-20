# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

# name = "Bonmin_jll"
# Bonmin_jll = "29cba6d7-6840-5cf2-a2fa-9bdfccfe29ea"
# name = "AmplNLWriter"
# AmplNLWriter = "7c4d4715-977e-5154-bfe0-e096adeac482"

using AmplNLWriter, Bonmin_jll

BONMIN_OPT = AmplNLWriter.Optimizer(Bonmin_jll.amplexe)
# MOI.set(BONMIN_OPT, MOI.Silent(), true)
# MOI.set(BONMIN_OPT, MOI.TimeLimitSec(), 60)
# BONMIN_OPT = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(BONMIN_OPT)

push!(solvers_nlp, (opt = BONMIN_OPT, mode = BilevelJuMP.ProductMode(1e-5)))
