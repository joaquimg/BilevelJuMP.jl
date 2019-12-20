module BilevelJuMP

using MathOptInterface
const MOI = MathOptInterface
const MOIU = MathOptInterface.Utilities
using JuMP
using Dualization


export
BilevelModel, UpperToLower, LowerToUpper, Upper, Lower, UpperOnly, LowerOnly

include("moi.jl")
include("jump.jl")

end