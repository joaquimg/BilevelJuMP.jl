module BilevelJuMP

using MathOptInterface
const MOI = MathOptInterface
const MOIU = MathOptInterface.Utilities
using JuMP
using Dualization
using LinearAlgebra
using SparseArrays

export
BilevelModel,
Upper, Lower, UpperOnly, LowerOnly,
DualOf, BilevelAffExpr, BilevelQuadExpr

@enum Level LOWER_BOTH UPPER_BOTH LOWER_ONLY UPPER_ONLY DUAL_OF_LOWER

include("intervals.jl")
include("moi.jl")
include("moi_utilities.jl")

include("jump.jl")
include("jump_variables.jl")
include("jump_constraints.jl")
include("jump_objective.jl")
include("jump_print.jl")
include("jump_attributes.jl")
include("jump_nlp.jl")
include("mibs.jl")
include("bilinear_linearization.jl")

end
