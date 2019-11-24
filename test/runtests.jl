using BilevelJuMP
using Test, MathOptInterface, JuMP, Dualization
# using MathOptFormat
using Gurobi


const MOI  = MathOptInterface
const MOIU = MathOptInterface.Utilities
const MOIB = MathOptInterface.Bridges
const MOIT = MathOptInterface.Test

solvers = NamedTuple{(:opt, :mode),Tuple{Any,Any}}[]
solvers_quad = NamedTuple{(:opt, :mode),Tuple{Any,Any}}[]

include("solvers/cbc.jl")
include("solvers/gurobi.jl")

include("moi.jl")
include("jump.jl")

@testset "Simple LP" begin
    for solver in solvers
        moi_01(solver.opt)
    end
end

@testset "Simple BLP" begin
    for solver in solvers
        moi_02(solver.opt, solver.mode)
    end
end

@testset "Simple BLP2" begin
    for solver in solvers
        moi_03(solver.opt, solver.mode)
    end
end

@testset "Simple BLP JuMP" begin
    for solver in solvers
        jump_01(solver.opt, solver.mode)
    end
end

@testset "Simple BLP2 JuMP" begin
    for solver in solvers
        jump_02(solver.opt, solver.mode)
    end
end

@testset "Simple 3 JuMP" begin
    for solver in solvers
        jump_03(solver.opt, solver.mode)
    end
end

@testset "Simple 4 JuMP" begin
    for solver in solvers
        jump_04(solver.opt, solver.mode)
    end
end

@testset "Simple 5 JuMP" begin
    for solver in solvers
        jump_05(solver.opt, solver.mode)
    end
end

@testset "Simple 3SAT JuMP" begin
    for solver in solvers
        jump_3SAT(solver.opt, solver.mode)
    end
end

# @testset "JuMP quad" begin
#     for solver in solvers_quad
#         jump_quad_01_a(solver.opt, solver.mode)
#     end
# end

@testset "Simple 6 JuMP" begin
    for solver in solvers
        jump_06(solver.opt, solver.mode)
    end
end

@testset "Simple 7 JuMP" begin
    for solver in solvers
        jump_07(solver.opt, solver.mode)
    end
end

@testset "Simple 8 JuMP" begin
    for solver in solvers
        jump_08(solver.opt, solver.mode)
    end
end

@testset "Simple 9 JuMP" begin
    for solver in solvers
        jump_09a(solver.opt, solver.mode)
        jump_09b(solver.opt, solver.mode)
    end
end


@testset "Simple 11 JuMP" begin
    for solver in solvers
        jump_11a(solver.opt, solver.mode)
        jump_11b(solver.opt, solver.mode)
    end
end

@testset "Simple 12 JuMP" begin
    for solver in solvers
        jump_12(solver.opt, solver.mode)
    end
end

@testset "Simple 14 JuMP" begin
    for solver in solvers
        jump_14(solver.opt, solver.mode)
    end
end
