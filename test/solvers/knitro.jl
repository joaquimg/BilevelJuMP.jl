# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

# name = "KNITRO"
# KNITRO = "67920dd8-b58e-52a8-8622-53c4cffbe346"

using KNITRO

KNITRO_OPT = KNITRO.Optimizer()
MOI.set(KNITRO_OPT, MOI.Silent(), true)
MOI.set(KNITRO_OPT, MOI.TimeLimitSec(), 60)
# KNITRO_OPT = MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(KNITRO_OPT)

push!(solvers_complements, (opt = KNITRO_OPT, mode = BilevelJuMP.ComplementMode()))
