# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

# name = "GAMS"
# GAMS = "1ca51c6a-1b4d-4546-9ae1-53e0a243ab12"

using GAMS

GAMS_OPT = GAMS.Optimizer()
MOI.set(GAMS_OPT, MOI.Silent(), true)
MOI.set(GAMS_OPT, MOI.TimeLimitSec(), 60)
# GAMS_OPT = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(GAMS_OPT)

push!(solvers_complements, (opt = GAMS_OPT, mode = BilevelJuMP.ComplementMode()))
