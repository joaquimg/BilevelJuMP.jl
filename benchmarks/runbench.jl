using Random
using Dates
using BilevelJuMP

using MathOptInterface
MOI = MathOptInterface
MOIU = MOI.Utilities

MAX_TIME = 600

include("svr.jl")
include("rand.jl")
include("forecast.jl")
include("toll.jl")

# do not believe in cbc feasible points:
# https://github.com/jump-dev/Cbc.jl/blob/a794c7592482de747f4c99e918db485574a7ea74/src/MOI_wrapper.jl#L683
# it is just LP feasible solution (not MIP)

mode = BilevelJuMP.SOS1Mode()

with_att = JuMP.optimizer_with_attributes

FA = BilevelJuMP.FortunyAmatMcCarlMode

using Xpress
using CPLEX
using Gurobi
using SCIP
using Cbc
using GLPK
using Ipopt
using KNITRO
using AmplNLWriter
using Mosek, MosekTools
using Juniper
using QuadraticToBinary

QB = QuadraticToBinary.Optimizer{Float64}
function cache(opt)
    return MOIU.CachingOptimizer(
        MOIU.UniversalFallback(MOIU.Model{Float64}()),
        opt,
    )
end

function cpx()
    s = CPLEX.Optimizer()
    MOI.set(s, MOI.RawParameter("CPXPARAM_TimeLimit"), MAX_TIME * 1)
    return s
end

# warm-up - precompilation
MAX_TIME = 0
bench_svr(1, 5, Gurobi.Optimizer, BilevelJuMP.SOS1Mode(), 1234)
bench_forecast(1, 5, Gurobi.Optimizer, BilevelJuMP.SOS1Mode(), 1234)
bench_toll(3, Gurobi.Optimizer, BilevelJuMP.SOS1Mode(), 1234)
MAX_TIME = 600

# comented due to KNTRO lic check
# const KN_OPT = KNITRO.Optimizer()#maxtimecpu = MAX_TIME*1.0)
# function new_knitro()
#     MOI.empty!(KN_OPT)
#     MOI.set(KN_OPT, MOI.RawParameter("maxtime_real"), MAX_TIME*1.0)
#     return KN_OPT
# end

SOLVERS = [
    #=
        SOS1
    =#
    (
        with_att(Gurobi.Optimizer, "TimeLimit" => MAX_TIME * 1),
        BilevelJuMP.SOS1Mode(),
        "gurobi_sos1",
    ),
    (
        with_att(CPLEX.Optimizer, "CPXPARAM_TimeLimit" => MAX_TIME * 1),
        BilevelJuMP.SOS1Mode(),
        "cplex_sos1",
    ),
    (
        with_att(
            Xpress.Optimizer,
            "MAXTIME" => -MAX_TIME * 1,
            "logfile" => "output.log",
        ),
        BilevelJuMP.SOS1Mode(),
        "xpress_sos1",
    ),
    (
        with_att(Cbc.Optimizer, "seconds" => MAX_TIME * 1.0),
        BilevelJuMP.SOS1Mode(),
        "cbc_sos1",
    ),
    (
        with_att(SCIP.Optimizer, "limits/time" => MAX_TIME * 1),
        BilevelJuMP.SOS1Mode(),
        "scip_sos1",
    ),
    #=
        indicator
    =#
    # (with_att(CPLEX.Optimizer, "CPXPARAM_TimeLimit" => MAX_TIME), BilevelJuMP.IndicatorMode(), "cplex_indc"),
    # # (with_att(Gurobi.Optimizer, "TimeLimit" => MAX_TIME), BilevelJuMP.IndicatorMode(), "gurobi_indc"), # not supporting indicator
    # (with_att(Xpress.Optimizer, "MAXTIME" => -MAX_TIME, "logfile" => "output.log"), BilevelJuMP.IndicatorMode(), "xpress_indc"),
    # (with_att(Cbc.Optimizer, "seconds" => MAX_TIME*1.0), BilevelJuMP.IndicatorMode(), "cbc_indc"),
    # # (with_att(SCIP.Optimizer, "limits/time" => MAX_TIME), BilevelJuMP.IndicatorMode(), "scip_indc"), # weird offset error
    #=
        Fortuny-Amat 10 (DONE)
    =#
    # (with_att(GLPK.Optimizer, "tm_lim" => MAX_TIME * 1_000), FA(primal_big_M = 10, dual_big_M = 10), "glpk_fa10"),
    # (with_att(Mosek.Optimizer, "MIO_MAX_TIME" => MAX_TIME * 1.0, "OPTIMIZER_MAX_TIME" => MAX_TIME * 1.0), FA(primal_big_M = 10, dual_big_M = 10), "mosek_fa10"), # no sos1
    # (with_att(Gurobi.Optimizer, "TimeLimit" => MAX_TIME*1), FA(primal_big_M = 10, dual_big_M = 10), "gurobi_fa10"),
    # (with_att(CPLEX.Optimizer, "CPXPARAM_TimeLimit" => MAX_TIME*1), FA(primal_big_M = 10, dual_big_M = 10), "cplex_fa10"), #TODO
    # (with_att(Xpress.Optimizer, "MAXTIME" => -MAX_TIME*1, "logfile" => "output.log"), FA(primal_big_M = 10, dual_big_M = 10), "xpress_fa10"),
    # (with_att(Cbc.Optimizer, "seconds" => MAX_TIME*1.0), FA(primal_big_M = 10, dual_big_M = 10), "cbc_fa10"),
    # (with_att(SCIP.Optimizer, "limits/time" => MAX_TIME*1), FA(primal_big_M = 10, dual_big_M = 10), "scip_fa10"),
    #=
        Fortuny-Amat 100 (DONE)
    =#
    # (with_att(GLPK.Optimizer, "tm_lim" => MAX_TIME * 1_000), FA(primal_big_M = 100, dual_big_M = 100), "glpk_fa100"),
    # (with_att(Mosek.Optimizer, "MIO_MAX_TIME" => MAX_TIME * 1.0, "OPTIMIZER_MAX_TIME" => MAX_TIME * 1.0), FA(primal_big_M = 100, dual_big_M = 100), "mosek_fa100"), # no sos1
    # (with_att(Gurobi.Optimizer, "TimeLimit" => MAX_TIME*1), FA(primal_big_M = 100, dual_big_M = 100), "gurobi_fa100"),
    # (with_att(CPLEX.Optimizer, "CPXPARAM_TimeLimit" => MAX_TIME*1), FA(primal_big_M = 100, dual_big_M = 100), "cplex_fa100"),
    # (with_att(Xpress.Optimizer, "MAXTIME" => -MAX_TIME*1, "logfile" => "output.log"), FA(primal_big_M = 100, dual_big_M = 100), "xpress_fa100"),
    # (with_att(Cbc.Optimizer, "seconds" => MAX_TIME*1.0), FA(primal_big_M = 100, dual_big_M = 100), "cbc_fa100"),
    # (with_att(SCIP.Optimizer, "limits/time" => MAX_TIME*1), FA(primal_big_M = 100, dual_big_M = 100), "scip_fa100"),
    #=
        Product BIN 10
    =#
    # (()->QB(GLPK.Optimizer(tm_lim=MAX_TIME*1_000),lb=-10,ub=10), BilevelJuMP.ProductMode(1e-7), "glpk_prod10"),
    # (()->QB(Mosek.Optimizer(MIO_MAX_TIME=MAX_TIME*1.0,OPTIMIZER_MAX_TIME=MAX_TIME*1.0),lb=-10,ub=10), BilevelJuMP.ProductMode(1e-7), "mosek_prod10"),
    # (()->QB(Gurobi.Optimizer(TimeLimit=MAX_TIME*1),lb=-10,ub=10), BilevelJuMP.ProductMode(1e-7), "gurobi_prod10"),
    # (()->QB(cpx(),lb=-10,ub=10), BilevelJuMP.ProductMode(1e-7), "cplex_prod10"), #TODO
    # (()->QB(Xpress.Optimizer(MAXTIME=-MAX_TIME*1),lb=-10,ub=10), BilevelJuMP.ProductMode(1e-7), "xpress_prod10"),
    # (()->QB(cache(Cbc.Optimizer(seconds=MAX_TIME*1.0)),lb=-10,ub=10), BilevelJuMP.ProductMode(1e-7), "cbc_prod10"),
    # (()->QB(SCIP.Optimizer(limits_time=MAX_TIME*1),lb=-10,ub=10), BilevelJuMP.ProductMode(1e-7), "scip_prod10"),
    #=
        Product BIN 100
    =#
    # (()->QB(GLPK.Optimizer(tm_lim=MAX_TIME*1_000),lb=-100,ub=100), BilevelJuMP.ProductMode(1e-7), "glpk_prod100"),
    # (()->QB(Mosek.Optimizer(MIO_MAX_TIME=MAX_TIME*1.0,OPTIMIZER_MAX_TIME=MAX_TIME*1.0),lb=-100,ub=100), BilevelJuMP.ProductMode(1e-7), "mosek_prod100"),
    # (()->QB(Gurobi.Optimizer(TimeLimit=MAX_TIME*1),lb=-100,ub=100), BilevelJuMP.ProductMode(1e-7), "gurobi_prod100"),
    # (()->QB(cpx(),lb=-100,ub=100), BilevelJuMP.ProductMode(1e-7), "cplex_prod100"),
    # (()->QB(Xpress.Optimizer(MAXTIME=-MAX_TIME*1),lb=-100,ub=100), BilevelJuMP.ProductMode(1e-7), "xpress_prod100"),
    # (()->QB(cache(Cbc.Optimizer(seconds=MAX_TIME*1.0)),lb=-100,ub=100), BilevelJuMP.ProductMode(1e-7), "cbc_prod100"),
    # (()->QB(SCIP.Optimizer(limits_time=MAX_TIME*1),lb=-100,ub=100), BilevelJuMP.ProductMode(1e-7), "scip_prod100"),
    #=
        PrimalDual BIN 10
    =#
    # (()->QB(GLPK.Optimizer(tm_lim=MAX_TIME*1_000),lb=-10,ub=10), BilevelJuMP.StrongDualityMode(), "glpk_sd10"),
    # (()->QB(Mosek.Optimizer(MIO_MAX_TIME=MAX_TIME*1.0,OPTIMIZER_MAX_TIME=MAX_TIME*1.0),lb=-10,ub=10), BilevelJuMP.StrongDualityMode(), "mosek_sd10"),
    # (()->QB(Gurobi.Optimizer(TimeLimit=MAX_TIME*1),lb=-10,ub=10), BilevelJuMP.StrongDualityMode(), "gurobi_sd10"),
    # (()->QB(cpx(),lb=-10,ub=10), BilevelJuMP.StrongDualityMode(), "cplex_sd10"),
    # (()->QB(Xpress.Optimizer(MAXTIME=-MAX_TIME*1),lb=-10,ub=10), BilevelJuMP.StrongDualityMode(), "xpress_sd10"),
    # (()->QB(cache(Cbc.Optimizer(seconds=MAX_TIME*1.0)),lb=-10,ub=10), BilevelJuMP.StrongDualityMode(), "cbc_sd10"), #TODO
    # (()->QB(SCIP.Optimizer(limits_time=MAX_TIME*1),lb=-10,ub=10), BilevelJuMP.StrongDualityMode(), "scip_sd10"),
    #=
        PrimalDual BIN 100
    =#
    # (()->QB(GLPK.Optimizer(tm_lim=MAX_TIME*1_000),lb=-100,ub=100), BilevelJuMP.StrongDualityMode(), "glpk_sd100"),
    # (()->QB(Mosek.Optimizer(MIO_MAX_TIME=MAX_TIME*1.0,OPTIMIZER_MAX_TIME=MAX_TIME*1.0),lb=-100,ub=100), BilevelJuMP.StrongDualityMode(), "mosek_sd100"),
    # (()->QB(Gurobi.Optimizer(TimeLimit=MAX_TIME*1),lb=-100,ub=100), BilevelJuMP.StrongDualityMode(), "gurobi_sd100"),
    # (()->QB(cpx(),lb=-100,ub=100), BilevelJuMP.StrongDualityMode(), "cplex_sd100"),
    # (()->QB(Xpress.Optimizer(MAXTIME=-MAX_TIME*1),lb=-100,ub=100), BilevelJuMP.StrongDualityMode(), "xpress_sd100"),
    # (()->QB(cache(Cbc.Optimizer(seconds=MAX_TIME*1.0)),lb=-100,ub=100), BilevelJuMP.StrongDualityMode(), "cbc_sd100"),
    # (()->QB(SCIP.Optimizer(limits_time=MAX_TIME*1),lb=-100,ub=100), BilevelJuMP.StrongDualityMode(), "scip_sd100"),
    #=
        PrimalDual NLP (DONE)
    =#
    # (with_att(Ipopt.Optimizer, "max_cpu_time" => MAX_TIME*1.0), BilevelJuMP.StrongDualityMode(), "ipopt_sd"),
    # (new_knitro, BilevelJuMP.StrongDualityMode(), "knitro_sd"),
    #=
        Product NLP (DONE)
    =#
    # (with_att(Ipopt.Optimizer, "max_cpu_time" => MAX_TIME*1.0), BilevelJuMP.ProductMode(1e-7), "ipopt_prod"),
    # (new_knitro, BilevelJuMP.ProductMode(1e-7), "knitro_prod"),
    #=
        Complemets
    =#
    # Only adds all at once # (new_knitro, BilevelJuMP.ComplementMode(), "knitro_comp"),
    #=
        Product global
    =#
    # (() -> AmplNLWriter.Optimizer("bonmin"), BilevelJuMP.ProductMode(1e-7), "bonmin_prod"),
    # (() -> AmplNLWriter.Optimizer("couenne"), BilevelJuMP.ProductMode(1e-7), "couenne_prod"),
    # (() -> AmplNLWriter.Optimizer("bonmin"), BilevelJuMP.StrongDualityMode(), "bonmin_sd"),
    # (() -> AmplNLWriter.Optimizer("couenne"), BilevelJuMP.StrongDualityMode(), "couenne_sd"),
]

PROBLEMS = [:SVR, :TOLL, :FORECAST, :RAND]

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
    (1, 10),
    (1, 10),
    (2, 10),
    # (  5,  10),
    # (  1, 100),
    # (  2, 100), # 600
    # (  5, 100),
    # ( 10, 100),
    # ( 20, 100),
    # ( 50, 100),
    # (  1,1000), # hard for prod10
    # (  2,1000),
    # (  5,1000),
    # # # ( 10,1000),
    # # # ( 20,1000),
    # # # ( 50,1000),
    # # # (100,1000),
    # # # (200,1000),
    # # # (500,1000),
    # # () for i in [1,2,5,10,20,50,100,200], j in [10, 100, 1000]
]

RAND = [
    # (rows, cols)
    (5, 5),
    (10, 5),
    (5, 10),
    (10, 10),
    # (  50,  10),
    # (  10,  50),
    # (  50,  50), # 600
    # ( 100,  50),
    # (  50, 100),
    # ( 100, 100),
    # # ( 500, 100),
    # # ( 100, 500),
    # # ( 500, 500),
    # # (1000, 500),
    # # ( 500,1000),
    # # (1000,1000),
    # # (5000,1000),
    # # (1000,5000),
    # # (5000,5000),
]

TOLL = [
    # nodes
    5,
    10,
    20,
    # 50, # hard for prod10
    # 100,
    # 200, # also massive on memory
    # 500, # 600 - broke gurobi
    # # 1000,
    # # 2000,
    # # 5000,
]

FORECAST = [
    # (products, sample_size)
    (1, 10),
    (2, 10),
    (5, 10),
    # (  1, 100),
    # (  2, 100), # hard for prod10
    # (  5, 100), # 600
    # ( 10, 100),
    # ( 20, 100),
    # ( 50, 100),
    # (  1,1000),
    # (  2,1000),
    # (  5,1000),
    # # ( 10,1000),
    # # ( 20,1000),
    # # ( 50,1000),
    # # (100,1000),
    # # (200,1000),
    # # (500,1000),
    # # () for i in [1,2,5,10,20,50,100,200], j in [10, 100, 1000]
]

function separator()
    println()
    println()
    println("============================================================")
    println("============================================================")
    println()
    return println()
end

function new_file()
    cd(dirname(@__FILE__))
    FILE = open("bench$(replace("$(now())",":"=>"_")).log", "w")
    println(
        FILE,
        "opt_mode, prob, inst, primal_status, termination_status, solve_time, build_time, lower_obj, upper_obj",
    )
    flush(FILE)
    return FILE
end
function newline(FILE, data, opt, prb, inst, seed)
    println(
        FILE,
        "$opt, $prb, $inst, $seed, $(data[1]),$(data[2]),$(data[3]),$(data[4]),$(data[5]),$(data[6]),$(data[7])",
    )
    return flush(FILE)
end
FILE = new_file()
for seed in SEEDS
    for (optimizer, mode, name) in SOLVERS
        if :SVR in PROBLEMS
            for (features, samples) in SVR
                separator()
                @show features, samples, seed, name
                separator()
                ret = bench_svr(features, samples, optimizer, mode, seed)
                newline(FILE, ret, name, :SVR, (features, samples), seed)
            end
        end
        if :RAND in PROBLEMS
            for (rows, cols) in RAND
                separator()
                @show rows, cols, seed, name
                separator()
                ret = bench_rand(rows, cols, 0.5, optimizer, mode, seed)
                newline(FILE, ret, name, :RAND, (rows, cols), seed)
            end
        end
        if :TOLL in PROBLEMS
            for nodes in TOLL
                separator()
                @show nodes, seed, name
                separator()
                ret = bench_toll(nodes, optimizer, mode, seed)
                newline(FILE, ret, name, :TOLL, (nodes,), seed)
            end
        end
        if :FORECAST in PROBLEMS
            for (products, samples) in FORECAST
                separator()
                @show products, samples, seed, name
                separator()
                ret = bench_forecast(products, samples, optimizer, mode, seed)
                newline(FILE, ret, name, :FORE, (products, samples), seed)
            end
        end
    end
end

close(FILE)

exit(0)
