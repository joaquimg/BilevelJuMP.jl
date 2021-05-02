module BilevelJuMP

using MathOptInterface
const MOI = MathOptInterface
const MOIU = MathOptInterface.Utilities
using JuMP
using Dualization
using LinearAlgebra
using IntervalArithmetic


export
BilevelModel, UpperToLower, LowerToUpper,
Upper, Lower, UpperOnly, LowerOnly,
DualOf

include("moi.jl")
include("moi_utilities.jl")

include("jump.jl")
include("jump_variables.jl")
include("jump_constraints.jl")
include("jump_objective.jl")
include("jump_print.jl")
include("jump_attributes.jl")
include("jump_nlp.jl")

end