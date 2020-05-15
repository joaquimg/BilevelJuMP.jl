using BilevelJuMP
using Test, MathOptInterface, JuMP, Dualization
# using MathOptFormat


const MOI  = MathOptInterface
const MOIU = MathOptInterface.Utilities
const MOIB = MathOptInterface.Bridges
const MOIT = MathOptInterface.Test

# TODO
# add JUMPExtension test

struct Config
    atol::Float64
    rtol::Float64
    bound_hint::Bool
    start_value::Bool
    function Config(;
        atol = 1e-6,
        rtol = 1e-6,
        bound_hint = false,
        start_value = false,
        )
        new(atol, rtol, bound_hint, start_value)
    end
end

config = Config()
config_low_tol = Config(atol = 1e-3, rtol = 1e-3)
CONFIG_3 = Config(atol = 1e-3, rtol = 1e-3)
CONFIG_3_hint = Config(atol = 1e-3, rtol = 1e-3, bound_hint = true)
CONFIG_4 = Config(atol = 1e-4, rtol = 1e-4)
CONFIG_5 = Config(atol = 1e-5, rtol = 1e-5)

OptModeType = NamedTuple{(:opt, :mode),Tuple{Any,Any}}

solvers = OptModeType[]
solvers_sos = OptModeType[]
solvers_quad = OptModeType[]
solvers_bin_exp = OptModeType[]
solvers_sos_quad = OptModeType[]
solvers_nlp = OptModeType[]
solvers_nlp_lowtol = OptModeType[]
solvers_sos_quad_bin = OptModeType[]

include("solvers/cbc.jl")
include("solvers/ipopt.jl")
# include("solvers/gurobi.jl")
# include("solvers/xpress.jl")
# include("solvers/path.jl")

include("moi.jl")
include("jump.jl")

@testset "Simple LP" begin
    for solver in solvers
        moi_01(solver.opt)
        moi_02(solver.opt, solver.mode)
        moi_03(solver.opt, solver.mode)
    end
end

@testset "Simple BLP JuMP" begin
    for solver in solvers_nlp
        jump_01(solver.opt, solver.mode, CONFIG_3)
        jump_01vec(solver.opt, solver.mode, CONFIG_3)
        jump_02(solver.opt, solver.mode)
        jump_03(solver.opt, solver.mode)
        jump_04(solver.opt, solver.mode)
        jump_05(solver.opt, solver.mode)
        #jump_3SAT(solver.opt, solver.mode)
        jump_06(solver.opt, solver.mode)
        jump_07(solver.opt, solver.mode, CONFIG_3)
        jump_08(solver.opt, solver.mode)
        jump_09a(solver.opt, solver.mode)
        jump_09b(solver.opt, solver.mode)
        jump_11a(solver.opt, solver.mode)
        jump_11b(solver.opt, solver.mode)
        jump_12(solver.opt, solver.mode)
        jump_14(solver.opt, solver.mode)
    end
    for solver in solvers_sos
        jump_01(solver.opt, solver.mode)
        jump_02(solver.opt, solver.mode)
        jump_03(solver.opt, solver.mode)
        jump_04(solver.opt, solver.mode)
        jump_05(solver.opt, solver.mode)
        jump_3SAT(solver.opt, solver.mode)
        jump_06(solver.opt, solver.mode)
        jump_07(solver.opt, solver.mode)
        jump_08(solver.opt, solver.mode)
        jump_09a(solver.opt, solver.mode) # fail on cbc positive SOS
        jump_09b(solver.opt, solver.mode)
        jump_11a(solver.opt, solver.mode)
        jump_11b(solver.opt, solver.mode)
        jump_12(solver.opt, solver.mode)
        jump_14(solver.opt, solver.mode)
    end
    for solver in solvers_bin_exp
        jump_01(solver.opt, solver.mode, CONFIG_3_hint)
        jump_01vec(solver.opt, solver.mode, CONFIG_3_hint)
        jump_02(solver.opt, solver.mode, CONFIG_3_hint)
        jump_03(solver.opt, solver.mode, CONFIG_3_hint)
    end
end

@testset "JuMP quad" begin
    for solver in solvers_quad
        jump_quad_01_a(solver.opt, solver.mode)
        jump_quad_01_b(solver.opt, solver.mode)
        jump_quad_01_c(solver.opt, solver.mode)
        jump_13_quad(solver.opt, solver.mode)
    end
end

@testset "Princeton Handbook of Test Problems" begin
    for solver in solvers_nlp
        jump_HTP_lin01(solver.opt, solver.mode)
        jump_HTP_lin02(solver.opt, solver.mode)
        jump_HTP_lin03(solver.opt, solver.mode)
        jump_HTP_lin04(solver.opt, solver.mode)
        jump_HTP_lin05(solver.opt, solver.mode) # broken on cbc linux on julia 1.0 and 1.2 but not 1.1 see: https://travis-ci.org/joaquimg/BilevelJuMP.jl/builds/619335351
        jump_HTP_lin06(solver.opt, solver.mode)
        jump_HTP_lin07(solver.opt, solver.mode, CONFIG_3)
        jump_HTP_lin08(solver.opt, solver.mode)
        jump_HTP_lin09(solver.opt, solver.mode)
        jump_HTP_lin10(solver.opt, solver.mode)
    end
    for solver in solvers_nlp
        jump_HTP_quad01(solver.opt, solver.mode)
        jump_HTP_quad02(solver.opt, solver.mode)
        jump_HTP_quad04(solver.opt, solver.mode, CONFIG_3)
        jump_HTP_quad05(solver.opt, solver.mode)
        jump_HTP_quad06(solver.opt, solver.mode, CONFIG_3)
        # jump_HTP_quad06b(solver.opt, solver.mode) # TODO weird results
        jump_HTP_quad07(solver.opt, solver.mode)
        jump_HTP_quad08(solver.opt, solver.mode) # not PSD
    end
    for solver in solvers_nlp
        jump_HTP_quad03(solver.opt, solver.mode)
        jump_HTP_quad09(solver.opt, solver.mode)
    end
    for solver in solvers_sos
        jump_HTP_lin01(solver.opt, solver.mode)
        jump_HTP_lin02(solver.opt, solver.mode)
        # jump_HTP_lin03(solver.opt, solver.mode) # failing cbc
        jump_HTP_lin04(solver.opt, solver.mode)
        println("Skipping HTP linear 05")
        # jump_HTP_lin05(solver.opt, solver.mode) # broken on cbc linux on julia 1.0 and 1.2 but not 1.1 see: https://travis-ci.org/joaquimg/BilevelJuMP.jl/builds/619335351
        jump_HTP_lin06(solver.opt, solver.mode)
        jump_HTP_lin07(solver.opt, solver.mode)
        jump_HTP_lin08(solver.opt, solver.mode)
        jump_HTP_lin09(solver.opt, solver.mode)
        jump_HTP_lin10(solver.opt, solver.mode)
    end
    for solver in solvers_sos_quad
        jump_HTP_quad01(solver.opt, solver.mode, CONFIG_5)
        jump_HTP_quad02(solver.opt, solver.mode)
        jump_HTP_quad04(solver.opt, solver.mode)
        jump_HTP_quad05(solver.opt, solver.mode)
        jump_HTP_quad06(solver.opt, solver.mode)
        # jump_HTP_quad06b(solver.opt, solver.mode)
        jump_HTP_quad07(solver.opt, solver.mode)
        # jump_HTP_quad08(solver.opt, solver.mode) # not PSD
    end
    for solver in solvers_sos
        jump_HTP_quad03(solver.opt, solver.mode)
        jump_HTP_quad09(solver.opt, solver.mode)
    end
end

@testset "Ferris educational" begin
    for solver in solvers
        jump_jointc1(solver.opt, solver.mode)
        jump_jointc2(solver.opt, solver.mode)
    end
end

@testset "Bard Efficient Point Example" begin
    for solver in solvers_sos
        jump_EffPointAlgo(solver.opt, solver.mode)
    end
end

@testset "SemiRandom" begin
    for solver in solvers_sos
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
        jump_DTMP_01_mod3vec(solver.opt, solver.mode)
        jump_DTMP_01_mod3(solver.opt, solver.mode)
    end
end

@testset "conejo2016" begin
    for solver in solvers_nlp
        jump_conejo2016(solver.opt, solver.mode)
    end
    for solver in solvers_sos_quad_bin
        jump_conejo2016(solver.opt, solver.mode, config, bounds = true)
    end
end

@testset "fanzeres2017" begin
    for solver in solvers_nlp
        jump_fanzeres2017(solver.opt, solver.mode)
    end
end

# require SOCtoNonConvexQuad bridge to work with Ipopt
@testset "Bilevel Conic JuMP NLP" begin
    for solver in solvers_nlp_lowtol
        #jump_conic01(solver.opt, solver.mode)
        #jump_conic02(solver.opt, solver.mode)
        #jump_conic03(solver.opt, solver.mode)
        #jump_conic04(solver.opt, solver.mode)
    end
end

@testset "Bilevel Conic JuMP SOC + MIP" begin
    for solver in solvers_sos_quad_bin
        jump_conic01(solver.opt, BilevelJuMP.ProductMode(), config, bounds = true)
        jump_conic02(solver.opt, BilevelJuMP.ProductMode(), config, bounds = true)
        jump_conic03(solver.opt, BilevelJuMP.ProductMode(), config, bounds = true)
        jump_conic04(solver.opt, BilevelJuMP.ProductMode(), config, bounds = true)
    end
end