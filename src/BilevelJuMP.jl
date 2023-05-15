module BilevelJuMP

import Reexport
Reexport.@reexport using JuMP

import MathOptInterface
const MOI = MathOptInterface
const MOIU = MathOptInterface.Utilities

import Dualization
import LinearAlgebra

export BilevelModel,
    Upper, Lower, UpperOnly, LowerOnly,
    DualOf,
    BilevelVariableRef, BilevelAffExpr, BilevelQuadExpr

"""
    Level

The level of a variable in a bilevel problem.
"""
@enum Level LOWER_BOTH UPPER_BOTH LOWER_ONLY UPPER_ONLY DUAL_OF_LOWER

@doc(
    "Indicates and object that is part of the lower level problem, but is shared with the upper level.",
    LOWER_BOTH
)

@doc(
    "Indicates and object that is part of the upper level problem, but is shared with the lower level.",
    UPPER_BOTH
)

@doc(
    "Indicates and object that is part of the lower level problem, but is not shared with the upper level.",
    LOWER_ONLY
)

@doc(
    "Indicates and object that is part of the upper level problem, but is not shared with the lower level.",
    UPPER_ONLY
)

@doc(
    "Indicates and object that is part of the dual of the lower level problem, and is shared with the upper level.",
    DUAL_OF_LOWER
)

include("intervals.jl")
include("moi.jl")
include("modes/sos1.jl")
include("modes/big_m.jl")
include("modes/complement.jl")
include("modes/indicator.jl")
include("modes/product.jl")
include("modes/strong_duality.jl")
include("modes/mixed.jl") # must be last because uses others
include("moi_utilities.jl")

include("jump.jl")
include("jump_variables.jl")
include("jump_constraints.jl")
include("jump_objective.jl")
include("jump_print.jl")
include("jump_attributes.jl")
include("jump_nlp.jl")
include("mibs.jl")

end
