using BilevelJuMP
using Test, MathOptInterface, JuMP, Dualization
# using MathOptFormat


const MOI  = MathOptInterface
const MOIU = MathOptInterface.Utilities
const MOIB = MathOptInterface.Bridges
const MOIT = MathOptInterface.Test

# TODO
# add JUMPExtension test

solvers = NamedTuple{(:opt, :mode),Tuple{Any,Any}}[]
solvers_quad = NamedTuple{(:opt, :mode),Tuple{Any,Any}}[]

include("solvers/cbc.jl")
# include("solvers/gurobi.jl")

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

@testset "Princeton Handbook of Test Problems" begin
    for solver in solvers
        jump_HTP_lin01(solver.opt, solver.mode)
        jump_HTP_lin02(solver.opt, solver.mode)
        jump_HTP_lin03(solver.opt, solver.mode)
        jump_HTP_lin04(solver.opt, solver.mode)
        println("Skipping HTP linear 05")
        # jump_HTP_lin05(solver.opt, solver.mode) # broken on cbc linux on julia 1.0 and 1.2 but not 1.1 see: https://travis-ci.org/joaquimg/BilevelJuMP.jl/builds/619335351
        jump_HTP_lin06(solver.opt, solver.mode)
        jump_HTP_lin07(solver.opt, solver.mode)
        jump_HTP_lin08(solver.opt, solver.mode)
        jump_HTP_lin09(solver.opt, solver.mode)
        jump_HTP_lin10(solver.opt, solver.mode)
    end
end

@testset "Ferris educational" begin
    for solver in solvers
        jump_jointc1(solver.opt, solver.mode)
        jump_jointc2(solver.opt, solver.mode)
    end
end

@testset "Bard Efficient Point Example" begin
    for solver in solvers
        jump_EffPointAlgo(solver.opt, solver.mode)
    end
end

@testset "SemiRandom" begin
    for solver in solvers
        jump_SemiRand(solver.opt, solver.mode)
    end
end

@testset "DNE" begin
    for solver in solvers
        jump_DTMP_01(solver.opt, solver.mode)
    end
end

@testset "DNE - modifications" begin
    for solver in solvers
        jump_DTMP_01_mod1(solver.opt, solver.mode)
        jump_DTMP_01_mod2_error(solver.opt, solver.mode)
    end
end