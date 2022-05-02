cd(@__DIR__)
using Pkg
Pkg.activate(".")
# Pkg.instantiate()

using Random
using Dates
using JuMP, BilevelJuMP

using MathOptInterface
MOIU = MOI.Utilities

MAX_TIME = 600

include("svr.jl")

using Xpress
using CPLEX
using Gurobi
using SCIP
using HiGHS
using Ipopt
# using KNITRO
using QuadraticToBinary

FA = BilevelJuMP.FortunyAmatMcCarlMode
QB = QuadraticToBinary.Optimizer{Float64}
cache(opt) = MOIU.CachingOptimizer(
    MOIU.UniversalFallback(MOIU.Model{Float64}()), opt)

# comented due to KNTRO lic check
# const KN_OPT = KNITRO.Optimizer()#maxtimecpu = MAX_TIME*1.0)
# function new_knitro()
#     MOI.empty!(KN_OPT)
#     MOI.set(KN_OPT, MOI.RawParameter("maxtime_real"), MAX_TIME*1.0)
#     return KN_OPT
# end

SOLVERS = [
    #=
        SOS1 (DONE)
    =#
    # (Gurobi.Optimizer, BilevelJuMP.SOS1Mode(), "gurobi_sos1"),
    # (CPLEX.Optimizer, BilevelJuMP.SOS1Mode(), "cplex_sos1"),
    # (Xpress.Optimizer, BilevelJuMP.SOS1Mode(), "xpress_sos1"),
    # (SCIP.Optimizer, BilevelJuMP.SOS1Mode(), "scip_sos1"),
    #=
        indicator (DONE)
    =#
    # (CPLEX.Optimizer, BilevelJuMP.IndicatorMode(), "cplex_indc"),
    # (Gurobi.Optimizer, BilevelJuMP.IndicatorMode(), "gurobi_indc"),
    # (Xpress.Optimizer, BilevelJuMP.IndicatorMode(), "xpress_indc"),
    # (SCIP.Optimizer, BilevelJuMP.IndicatorMode(), "scip_indc"),
    #=
        Fortuny-Amat 10 (DONE)
    =#
    # (Gurobi.Optimizer, FA(primal_big_M = 10, dual_big_M = 10), "gurobi_fa10"),
    # (CPLEX.Optimizer, FA(primal_big_M = 10, dual_big_M = 10), "cplex_fa10"), #TODO
    # (Xpress.Optimizer, FA(primal_big_M = 10, dual_big_M = 10), "xpress_fa10"),
    # (SCIP.Optimizer, FA(primal_big_M = 10, dual_big_M = 10), "scip_fa10"),
    # (HiGHS.Optimizer, FA(primal_big_M = 10, dual_big_M = 10), "highs_fa10"),
    #=
        Fortuny-Amat 100 (DONE)
    =#
    # (Gurobi.Optimizer, FA(primal_big_M = 100, dual_big_M = 100), "gurobi_fa100"),
    # (CPLEX.Optimizer, FA(primal_big_M = 100, dual_big_M = 100), "cplex_fa100"), #TODO
    # (Xpress.Optimizer, FA(primal_big_M = 100, dual_big_M = 100), "xpress_fa100"),
    # (SCIP.Optimizer, FA(primal_big_M = 100, dual_big_M = 100), "scip_fa100"),
    # (HiGHS.Optimizer, FA(primal_big_M = 100, dual_big_M = 100), "highs_fa100"),
    #=
        Product BIN 100 (TODO)
    =#
    (()->QB(Gurobi.Optimizer(),lb=-100,ub=100), BilevelJuMP.ProductMode(1e-7), "gurobi_prod100"),
    (()->QB(CPLEX.Optimizer(),lb=-100,ub=100), BilevelJuMP.ProductMode(1e-7), "cplex_prod100"), #TODO
    (()->QB(Xpress.Optimizer(),lb=-100,ub=100), BilevelJuMP.ProductMode(1e-7), "xpress_prod100"),
    (()->QB(SCIP.Optimizer(),lb=-100,ub=100), BilevelJuMP.ProductMode(1e-7), "scip_prod100"),
    (()->QB(HiGHS.Optimizer(),lb=-100,ub=100), BilevelJuMP.ProductMode(1e-7), "highs_prod100"),
    #=
        PrimalDual BIN 100 (TODO)
    =#
    (()->QB(Gurobi.Optimizer(),lb=-100,ub=100), BilevelJuMP.StrongDualityMode(), "gurobi_sd100"),
    (()->QB(CPLEX.Optimizer(),lb=-100,ub=100), BilevelJuMP.StrongDualityMode(), "cplex_sd100"),
    (()->QB(Xpress.Optimizer(),lb=-100,ub=100), BilevelJuMP.StrongDualityMode(), "xpress_sd100"),
    (()->QB(SCIP.Optimizer(),lb=-100,ub=100), BilevelJuMP.StrongDualityMode(), "scip_sd100"),
    (()->QB(HiGHS.Optimizer(),lb=-100,ub=100), BilevelJuMP.StrongDualityMode(), "highs_sd100"),
    #=
        PrimalDual NLP (TODO)
    =#
    (Ipopt.Optimizer, BilevelJuMP.StrongDualityMode(), "ipopt_sd"),
    (optimizer_with_attributes(Gurobi.Optimizer, "NonConvex" => 2), BilevelJuMP.StrongDualityMode(), "gurobnc_sd"),
    # (new_knitro, BilevelJuMP.StrongDualityMode(), "knitro_sd"),
    #=
    Product NLP (TODO)
    =#
    (Ipopt.Optimizer, BilevelJuMP.ProductMode(1e-7), "ipopt_prod"),
    (optimizer_with_attributes(Gurobi.Optimizer, "NonConvex" => 2), BilevelJuMP.ProductMode(1e-7), "gurobnc_sd"),
    # (new_knitro, BilevelJuMP.ProductMode(1e-7), "knitro_prod"),
    #=
        Complemets (TODO)
    =#
    # Only adds all at once # (new_knitro, BilevelJuMP.ComplementMode(), "knitro_comp"),
]

SEEDS = [
    1234,
    # 2345,
    # 3456,
    # 4567,
    # 5678,
    # 6789,
    # 7890,
    # 8901,
    # 9012,
    # 0123,
]

SVR = [
    # (features, sample_size)
    (  1,  10),
    (  1,  10),
    (  2,  10),
    (  5,  10),
    (  1, 100),
    (  2, 100), # 600
    (  5, 100),
    ( 10, 100),
    ( 20, 100),
    ( 50, 100),
    (  1,1000), # hard for prod10
    (  2,1000),
    (  5,1000),
    # # # ( 10,1000),
    # # # ( 20,1000),
    # # # ( 50,1000),
    # # # (100,1000),
    # # # (200,1000),
    # # # (500,1000),
    # # () for i in [1,2,5,10,20,50,100,200], j in [10, 100, 1000]
]

function separator()
    println("\n\n============================================================")
    println("============================================================\n\n")
end

function new_file()
    cd(dirname(@__FILE__))
    FILE = open("bench$(replace("$(now())",":"=>"_")).log", "w")
    println(FILE, "opt_mode, prob, inst, seed, primal_status, termination_status, solve_time, build_time, lower_obj, upper_obj")
    flush(FILE)
    return FILE
end
function newline(FILE, data, opt, prb, inst, seed)
    println(FILE, "$opt, $prb, $inst, $seed, $(data[1]),$(data[2]),$(data[3]),$(data[4]),$(data[5]),$(data[6]),$(data[7])")
    flush(FILE)
end
FILE = new_file()
for seed in SEEDS
    for (optimizer, mode, name) in SOLVERS
        for (features, samples) in SVR
            GC.gc()
            separator()
            @show features, samples, seed, name
            separator()
            try
                ret = bench_svr(features, samples, optimizer, mode, seed)
                newline(FILE, ret, name, :SVR, (features, samples), seed)
            catch e
                @show e
                println(FILE, "$name, SVR, ($features, $samples), $seed, FATAL_ERROR, FATAL_ERROR, 999, 999, NaN, NaN")
                flush(FILE)
            end
        end
    end
end

close(FILE)

# exit(0)