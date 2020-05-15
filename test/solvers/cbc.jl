using Cbc
using QuadraticToBinary

const CBC_OPTIMIZER = Cbc.Optimizer()
MOI.set(CBC_OPTIMIZER, MOI.Silent(), true)
const CBC_CACHE = MOIU.UniversalFallback(MOIU.Model{Float64}())
const CBC_CACHED = MOIU.CachingOptimizer(CBC_CACHE, CBC_OPTIMIZER)
const CBC_BRIDGED = MOI.Bridges.full_bridge_optimizer(CBC_CACHED, Float64)

push!(solvers, (opt = CBC_BRIDGED, mode = BilevelJuMP.SOS1Mode()))
push!(solvers_sos, (opt = CBC_BRIDGED, mode = BilevelJuMP.SOS1Mode()))

push!(solvers_bin_exp, (
    opt = QuadraticToBinary.Optimizer{Float64}(CBC_BRIDGED),
    mode = BilevelJuMP.StrongDualityInequalityMode{Float64}(1e-9)))
MOI.set(solvers_bin_exp[end].opt, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

push!(solvers_bin_exp, (
    opt = QuadraticToBinary.Optimizer{Float64}(CBC_BRIDGED),
    mode = BilevelJuMP.StrongDualityEqualityMode()))
MOI.set(solvers_bin_exp[end].opt, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)

# push!(solvers_sos_quad, (
#     opt = QuadraticToBinary.Optimizer{Float64}(CBC_BRIDGED),
#     mode = BilevelJuMP.SOS1Mode()))
# MOI.set(solvers_bin_exp[end].opt, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)