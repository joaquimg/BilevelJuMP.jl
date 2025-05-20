# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

using PATH

const PTH_OPTIMIZER = PATH.Optimizer()#output = "no")
# MOI.set(PTH_OPTIMIZER, MOI.Silent(), true)
const PTH_CACHE = MOIU.UniversalFallback(MOIU.Model{Float64}())
const PTH_CACHED = MOIU.CachingOptimizer(PTH_CACHE, PTH_OPTIMIZER)
const PTH_BRIDGED = MOI.Bridges.full_bridge_optimizer(PTH_CACHED, Float64)

push!(solvers_nlp, (opt = PTH_BRIDGED, mode = BilevelJuMP.ComplementMode{Float64}(1e-9)))

push!(solvers, (opt = PTH_BRIDGED, mode = BilevelJuMP.ComplementMode{Float64}(1e-9)))
push!(solvers_quad, (opt = PTH_BRIDGED, mode = BilevelJuMP.ComplementMode{Float64}(1e-9)))