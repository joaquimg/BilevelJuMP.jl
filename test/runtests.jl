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
CONFIG_1 = Config(atol = 1e-1, rtol = 1e-2)
CONFIG_2 = Config(atol = 1e-2, rtol = 1e-2)
CONFIG_3 = Config(atol = 1e-3, rtol = 1e-3)
CONFIG_3_start = Config(atol = 1e-3, rtol = 1e-3, start_value = true)
CONFIG_3_hint = Config(atol = 1e-3, rtol = 1e-3, bound_hint = true)
CONFIG_4 = Config(atol = 1e-4, rtol = 1e-4)
CONFIG_5 = Config(atol = 1e-5, rtol = 1e-5)

OptModeType = NamedTuple{(:opt, :mode),Tuple{Any,Any}}

solvers = OptModeType[]
solvers_cached = OptModeType[]
solvers_sos = OptModeType[]
solvers_indicator = OptModeType[]
solvers_quad = OptModeType[]
solvers_bin_exp = OptModeType[]
solvers_sos_quad = OptModeType[]
solvers_nlp = OptModeType[]
solvers_nlp_sd = OptModeType[]
solvers_nlp_sd_e = OptModeType[]
solvers_nlp_sd_i = OptModeType[]
solvers_nlp_lowtol = OptModeType[]
solvers_sos_quad_bin = OptModeType[]
solvers_fa_quad_bin = OptModeType[]
solvers_fa_quad_bin_mixed = OptModeType[]
solvers_fa = OptModeType[]
solvers_fa2 = OptModeType[]

include("solvers/cbc.jl")
include("solvers/ipopt.jl")
# include("solvers/gurobi.jl")
# include("solvers/xpress.jl")
# include("solvers/path.jl")

include("moi.jl")
include("jump.jl")
include("jump_unit.jl")

@testset "BilevelJuMP tests" begin

@testset "Unit" begin
    jump_display()
    jump_objective()
    jump_bounds()
    jump_attributes()
    mixed_mode_unit()
    jump_constraints()
    jump_variables()
    for solver in solvers_sos
        invalid_lower_objective(solver.opt, solver.mode)
        jump_display_solver(solver.opt, solver.mode)
        invalid_optimizer(solver.opt, solver.mode)
        jump_objective_solver(solver.opt, solver.mode)
        jump_attributes_solver(solver.opt, solver.mode)
    end
end

@testset "Simple LP" begin
    for solver in solvers_cached
        moi_01(solver.opt)
        moi_02(solver.opt, solver.mode)
        moi_03(solver.opt, solver.mode)
    end
end

@testset "Simple BLP JuMP" begin
    for solver in solvers_nlp
        jump_01(solver.opt, solver.mode, CONFIG_3)
        jump_01vec(solver.opt, solver.mode, CONFIG_3)
        # jump_02(solver.opt, solver.mode) # numerical isntability in ipopt
        jump_03(solver.opt, solver.mode, CONFIG_3_start)
        jump_03_vec(solver.opt, solver.mode, CONFIG_3_start)
        jump_04(solver.opt, solver.mode, CONFIG_3_start)
        jump_05(solver.opt, solver.mode)
        #jump_3SAT(solver.opt, solver.mode)
        jump_06(solver.opt, solver.mode)
        jump_06_sv(solver.opt, solver.mode)
        jump_07(solver.opt, solver.mode, CONFIG_2)
        jump_08(solver.opt, solver.mode, CONFIG_3_start)
        jump_09a(solver.opt, solver.mode)
        jump_09b(solver.opt, solver.mode)
        jump_11a(solver.opt, solver.mode)
        jump_11b(solver.opt, solver.mode)
        jump_12(solver.opt, solver.mode)
        jump_14(solver.opt, solver.mode)
    end
    for solver in solvers_sos
        jump_01_mixed(solver.opt)
    end
    for solver in solvers_sos
        jump_01(solver.opt, solver.mode)
        jump_02(solver.opt, solver.mode)
        jump_03(solver.opt, solver.mode)
        jump_04(solver.opt, solver.mode)
        jump_05(solver.opt, solver.mode)
        jump_3SAT(solver.opt, solver.mode)
        jump_06(solver.opt, solver.mode)
        jump_06_sv(solver.opt, solver.mode)
        jump_07(solver.opt, solver.mode)
        jump_08(solver.opt, solver.mode)
        jump_09a(solver.opt, solver.mode) # fail on cbc positive SOS
        jump_09b(solver.opt, solver.mode)
        jump_11a(solver.opt, solver.mode)
        jump_11b(solver.opt, solver.mode)
        jump_12(solver.opt, solver.mode)
        jump_14(solver.opt, solver.mode)
        #
        jump_16(solver.opt, solver.mode)
    end
    for solver in solvers_bin_exp
        jump_01(solver.opt, solver.mode, CONFIG_3_hint)
        jump_01vec(solver.opt, solver.mode, CONFIG_3_hint)
        jump_02(solver.opt, solver.mode, CONFIG_3_hint)
        jump_03(solver.opt, solver.mode, CONFIG_3_hint)
    end
    for solver in solvers_fa
        jump_01(solver.opt, solver.mode, CONFIG_3_hint)
        jump_01vec(solver.opt, solver.mode, CONFIG_3_hint)
        jump_02(solver.opt, solver.mode, CONFIG_3_hint)
        jump_03(solver.opt, solver.mode, CONFIG_3_hint)
    end
    for solver in solvers_fa2
        jump_01(solver.opt, solver.mode, CONFIG_3)
        jump_01vec(solver.opt, solver.mode, CONFIG_3)
        jump_02(solver.opt, solver.mode, CONFIG_3)
        jump_03(solver.opt, solver.mode, CONFIG_3)
    end
    for solver in solvers_indicator
        jump_01(solver.opt, solver.mode)
        jump_01vec(solver.opt, solver.mode)
        # jump_02(solver.opt, solver.mode) # fail cbc - pass xpress 8.9
        jump_03(solver.opt, solver.mode)
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
    for solver in vcat(solvers_nlp, solvers_nlp_sd)
        jump_HTP_lin01(solver.opt, solver.mode, CONFIG_3_start)
        # jump_HTP_lin02(solver.opt, solver.mode, CONFIG_4)
        jump_HTP_lin03(solver.opt, solver.mode)
        jump_HTP_lin03_vec(solver.opt, solver.mode)
        jump_HTP_lin04(solver.opt, solver.mode)
        jump_HTP_lin05(solver.opt, solver.mode) # broken on cbc linux on julia 1.0 and 1.2 but not 1.1 see: https://travis-ci.org/joaquimg/BilevelJuMP.jl/builds/619335351
        jump_HTP_lin06(solver.opt, solver.mode)
        jump_HTP_lin07(solver.opt, solver.mode, CONFIG_2)
        jump_HTP_lin09(solver.opt, solver.mode)
        # jump_HTP_lin10(solver.opt, solver.mode)
    end
    for solver in solvers_nlp
        jump_HTP_lin02(solver.opt, solver.mode)
        jump_HTP_lin10(solver.opt, solver.mode)
    end
    for solver in solvers_nlp_sd
        jump_HTP_lin02(solver.opt, solver.mode, CONFIG_2)
    end
    for solver in solvers_nlp_sd_i
        jump_HTP_lin08(solver.opt, solver.mode, CONFIG_1)
        jump_HTP_lin10(solver.opt, solver.mode, CONFIG_4)
    end
    for solver in solvers_nlp_sd_e
        jump_HTP_lin08(solver.opt, solver.mode, CONFIG_4)
        # jump_HTP_lin10(solver.opt, solver.mode)
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

@testset "equilibrium" begin
    for solver in solvers_nlp
        jump_conejo2016(solver.opt, solver.mode)
        jump_fanzeres2017(solver.opt, solver.mode)
        # jump_eq_price(solver.opt, solver.mode)
    end
    for solver in solvers_nlp_sd
        jump_conejo2016(solver.opt, solver.mode)
        # jump_fanzeres2017(solver.opt, solver.mode)
        # jump_eq_price(solver.opt, solver.mode)
    end
    for solver in solvers_sos_quad_bin
        # jump_conejo2016(solver.opt, solver.mode, config, bounds = true) # fail travis on cbc
        # jump_fanzeres2017(solver.opt, solver.mode)
        # jump_eq_price(solver.opt, solver.mode) # fail travis on cbc
    end
    for solver in solvers_fa_quad_bin
        jump_conejo2016(solver.opt, solver.mode, config, bounds = true)
        # jump_fanzeres2017(solver.opt, solver.mode)
        jump_eq_price(solver.opt, solver.mode)
    end
end

@testset "Fruits" begin
    for solver in solvers_fa2
        jump_fruits(solver.opt, solver.mode, CONFIG_4, 0.05)
        jump_fruits(solver.opt, solver.mode, CONFIG_4, 0.09)
    end
end

@testset "Bilevel Conic JuMP NLP" begin
    for solver in solvers_nlp_lowtol
        jump_conic01(solver.opt, solver.mode)
        jump_conic02(solver.opt, solver.mode)
        jump_conic03(solver.opt, solver.mode)
        jump_conic04(solver.opt, solver.mode)
    end
end

@testset "Bilevel Conic JuMP MIP" begin
    for solver in solvers_fa_quad_bin_mixed
        # @time jump_conic01(solver.opt, solver.mode, config, bounds = true)
        @time jump_conic02(solver.opt, solver.mode, config, bounds = true)
        @time jump_conic03(solver.opt, solver.mode, config, bounds = true)
        @time jump_conic04(solver.opt, solver.mode, config, bounds = true)
    end
end

end