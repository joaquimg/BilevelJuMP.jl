# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

# name = "BARON"
# add BARON

using BARON

BARON_OPT = BARON.Optimizer()
# MOI.set(BARON_OPT, MOI.Silent(), true)
# MOI.set(BARON_OPT, MOI.TimeLimitSec(), 60)
# BARON_OPT = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(BARON_OPT)

push!(solvers_nlp, (opt = BARON_OPT, mode = BilevelJuMP.ProductMode(1e-5)))
