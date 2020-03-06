module BilevelJuMP

using MathOptInterface
const MOI = MathOptInterface
const MOIU = MathOptInterface.Utilities
using JuMP
using Dualization
using LinearAlgebra


export
BilevelModel, UpperToLower, LowerToUpper,
Upper, Lower, UpperOnly, LowerOnly,
DualOf

include("moi.jl")
include("jump.jl")

end