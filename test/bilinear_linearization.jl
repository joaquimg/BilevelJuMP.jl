
using BilevelJuMP
# using Ipopt
# using Cbc
# using CPLEX
using Gurobi
using MathOptInterface
using Test
const MOI = MathOptInterface
using JuMP
using SparseArrays
using LinearAlgebra


## proposal Example

cder = 1
clmp = 1
ci = 1
d1 = 1
d2 = 2


# MILP Program
model = BilevelModel(
    Gurobi.Optimizer, 
    mode = BilevelJuMP.SOS1Mode(), 
    linearize_bilinear_upper_terms=true
)
@variables(Upper(model), begin
    10 >= x0 >= 0
    10 >= xe >= 0
end)

@variables(Lower(model), begin
    10 >= ye
    10 >= yi
    10 >= yder
end)

@constraints(Lower(model), begin
    ye >= 0
    yi >= 0
    yder >= 0
end)

@objective(Lower(model), Min, cder * yder + ci * yi - xe * ye)
@constraint(Lower(model), loadbal, yi - ye + yder == d1)

@variable(Upper(model), lambda, DualOf(loadbal))

@objective(Upper(model), Min, clmp * x0 + lambda * ye)
@constraint(Upper(model), x0 + ye - yi - d2 == 0)
@constraint(Upper(model), [ye, yi] in MOI.SOS1([1.0, 2.0]))

optimize!(model)

@expression(model, LLcost, cder * yder + ci * yi - xe * ye)
@test objective_value(model) ≈ 2.0
@test value(LLcost) ≈ 1.0
@test value(yi) ≈ 0.0
@test value(ye) ≈ 0.0
@test value(yder) ≈ 1.0
@test value(x0) ≈ 2.0


# MIQP
model2 = BilevelModel(Gurobi.Optimizer, mode = BilevelJuMP.SOS1Mode())
@variables(Upper(model2), begin
    10 >= x0 >= 0
    10 >= xe >= 0
end)

@variables(Lower(model2), begin
    10 >= ye
    10 >= yi
    10 >= yder
end)

@constraints(Lower(model2), begin
    ye >= 0
    yi >= 0
    yder >= 0
end)

@objective(Lower(model2), Min, cder * yder + ci * yi - xe * ye)
@constraint(Lower(model2), loadbal, yi - ye + yder == d1)

@variable(Upper(model2), lambda, DualOf(loadbal))

@objective(Upper(model2), Min, clmp * x0 + lambda * ye)
@constraint(Upper(model2), x0 + ye - yi - d2 == 0)
@constraint(Upper(model2), [ye, yi] in MOI.SOS1([1.0, 2.0]))

optimize!(model2)

@expression(model2, LLcost, cder * yder + ci * yi - xe * ye)
@test objective_value(model2) ≈ 2.0
@test value(LLcost) ≈ 1.0
@test value(yi) ≈ 0.0
@test value(ye) ≈ 0.0
@test value(yder) ≈ 1.0
@test value(x0) ≈ 2.0




#= 
        explicit KKT LL with bilinear UL:
=# 

model = JuMP.Model(Gurobi.Optimizer)
set_optimizer_attribute(model, "NonConvex", 2)
set_optimizer_attribute(model, "MIPGap", 1e-2)

@variables(model, begin
    10 >= x0 >= 0
    10 >= xe >= 0
    # 10 >= xi >= -10
    10 >= ye >= 0
    10 >= yi >= 0
    10 >= yder >= 0
    10 >= lambda >= -10
    10 >= mu_e_up >= 0
    10 >= mu_e_lo >= 0
    10 >= mu_i_up >= 0
    10 >= mu_i_lo >= 0
    10 >= mu_der_up >= 0
    10 >= mu_der_lo >= 0
end)

# @objective(model, Min, clmp * x0 ) + lambda * ye)
## bilinear problem results in costs UL=2, LL=1 and xe = 1, yder = 1, x0 = 2

@objective(model, Min, clmp * x0 + cder * yder + 10 * mu_der_up + ci * yi + 10 * mu_i_up - d1 * lambda)
## linearized objective gives same cost (UL=2, LL=1) but with xe = 1 and yder = 1, x0 = 2

@constraints(model, begin
    # x0 >= xi
    x0 - yi + ye == d2
    yi - ye + yder == d1
    -xe + lambda + mu_e_up == 0 + mu_e_lo
    ci - lambda + mu_i_up == 0  + mu_i_lo
    cder - lambda + mu_der_up == 0  + mu_der_lo
end)
@constraint(model, [ye, yi] in MOI.SOS1([1.0, 2.0]))
@constraint(model, [ye, mu_e_up] in MOI.SOS1([1.0, 2.0]))
@constraint(model, [ye, mu_e_lo] in MOI.SOS1([1.0, 2.0]))
@constraint(model, [yi, mu_i_up] in MOI.SOS1([1.0, 2.0]))
@constraint(model, [yi, mu_i_lo] in MOI.SOS1([1.0, 2.0]))
@constraint(model, [yder, mu_der_up] in MOI.SOS1([1.0, 2.0]))
@constraint(model, [yder, mu_der_lo] in MOI.SOS1([1.0, 2.0]))
optimize!(model)

@expression(model, LLcost, cder * yder + ci * yi - xe * ye)
@test objective_value(model) ≈ 2.0
@test value(LLcost) ≈ 1.0
@test value(yder) ≈ 1
@test value(LLcost) ≈ 1.0
@test value(yi) ≈ 0.0
@test value(ye) ≈ 0.0
@test value(yder) ≈ 1.0
@test value(x0) ≈ 2.0

#=
BilevelJuMP model indicates that it has only 4 SOS1 constraints:
julia> println(model.solver.model)
    sense  : minimize
    number of variables             = 12
    number of linear constraints    = 8
    number of quadratic constraints = 0
    number of sos constraints       = 4
    number of non-zero coeffs       = 19
    number of non-zero qp objective terms  = 1
    number of non-zero qp constraint terms = 0

So I assumed that it is not modeling the lower mu SOS1 constraints, ran Gurobi NLP again, and get
the result that matches BilevelJuMP solution (UL cost 0, LL cost 3)

So it seems that BilevelJuMP (actually it _is_ Dualization.jl dual_problem.primal_dual_map in BilevelJuMP moi.jl)
 is not including the complementary constraints for lower bounds of zero 
(but is including the dual variables for lower bounds of zero in the dual constraints, which results in the odd solution of UL cost 0, LL cost 3)

Bug in v 0.3.5 of Dualization.jl see https://github.com/jump-dev/Dualization.jl/issues/123 and address by checking out master branch of BilevelJuMP (Done)
=# 





#= 
        test jump_conejo2016 without linearization
=#

optimizer = Gurobi.Optimizer()
model = BilevelModel(()->optimizer)
set_optimizer_attribute(model, "NonConvex", 2)

bounds=true
@variable(Upper(model), x, start = 50)
if bounds
    @variable(Lower(model), -1 <= y[i=1:3] <= 300, start = [50, 150, 0][i])
else
    @variable(Lower(model), y[i=1:3], start = [50, 150, 0][i])
end
# 2 and 3 are lower only

@constraint(Upper(model), lb0, x >= 0)
@constraint(Upper(model), ub0, x <= 250)

@objective(Lower(model), Min, 10y[1] + 12y[2] + 15y[3])

@constraint(Lower(model), b, y[1] + y[2] + y[3] == 200)
@constraint(Lower(model), ub1, y[1] <= x)
@constraint(Lower(model), ub2, y[2] <= 150)
@constraint(Lower(model), ub3, y[3] <= 100)
@constraint(Lower(model), lb[i=1:3], y[i] >= 0)
if bounds
    @variable(Upper(model), 0 <= lambda <= 20, DualOf(b), start = 15)
else
    @variable(Upper(model), lambda, DualOf(b), start = 15)
end

@objective(Upper(model), Min, 40_000x + 8760*(10y[1]-lambda*y[1]))


optimize!(model)

@test objective_value(model) ≈ -190_000 atol=1e-1 rtol=1e-2
@test value(x) ≈ 50 atol=1e-3 rtol=1e-2
@test value.(y) ≈ [50, 150, 0] atol=1e-3 rtol=1e-2
@test value(lambda) ≈ 15 atol=1e-3 rtol=1e-2
@test value(40_000x + 8760*(10y[1]-lambda*y[1])) == -190000.0



#=
w/o y[1] <= x solution is: 
0 objective value y = [200, 0, 0], lambda = 10, x = 0
but according to linearized solution, could have lower objective value with lambda = 20,
but lamda is marginal cost of violating equality y[1] + y[2] + y[3] == 200, i.e. cost of y[1]
=#

#= 
        or with linearization:
=#


optimizer = Gurobi.Optimizer()
model = BilevelModel(()->optimizer, linearize_bilinear_upper_terms=true)
bounds = true
@variable(Upper(model), x, start = 50)
if bounds
    @variable(Lower(model), -1 <= y[i=1:3] <= 300, start = [50, 150, 0][i])
else
    @variable(Lower(model), y[i=1:3], start = [50, 150, 0][i])
end
# 2 and 3 are lower only

@constraint(Upper(model), lb0, x >= 0)
@constraint(Upper(model), ub0, x <= 250)

@objective(Lower(model), Min, 10y[1] + 12y[2] + 15y[3])

@constraint(Lower(model), b, y[1] + y[2] + y[3] == 200)
@constraint(Lower(model), ub1, y[1] <= x)
@constraint(Lower(model), ub2, y[2] <= 150)
@constraint(Lower(model), ub3, y[3] <= 100)
@constraint(Lower(model), ub4, y[3] <= 300)
@constraint(Lower(model), lb[i=1:3], y[i] >= 0)
if bounds
    @variable(Upper(model), 0 <= lambda <= 20, DualOf(b), start = 15)
else
    @variable(Upper(model), lambda, DualOf(b), start = 15)
end

@objective(Upper(model), Min, 40_000x + 8760*(10y[1]-lambda*y[1]))

BilevelJuMP.set_dual_lower_bound(ub4, 0)
BilevelJuMP.set_dual_lower_bound(ub3, 0)
BilevelJuMP.set_dual_lower_bound(ub2, 0)
BilevelJuMP.set_dual_lower_bound(ub1, 0)
for cref in lb
    BilevelJuMP.set_dual_lower_bound(cref, 0)
end

optimize!(model)


@test value(x) ≈ 50 atol=1e-3 rtol=1e-2
@test value.(y) ≈ [50, 150, 0] atol=1e-3 rtol=1e-2
@test value(lambda) ≈ 15 atol=1e-3 rtol=1e-2
@test value(40_000x + 8760*(10y[1]-lambda*y[1])) == -190000.0

# TODO why doesn't objective_value match the Upper objective value?
# @test objective_value(model) ≈ -190_000 atol=1e-1 rtol=1e-2
# has to do with variables at bounds? y[1] = x = 50, y[2] = 150 (its UB)
# yes it seems related to dual variables for variable bounds, if I set the mu variables to >= 0 then the objective_value matches the non linear result:
@test value(40_000x + 8760*(10y[1]-lambda*y[1])) == objective_value(model)
#=
julia> (objective_value(model) + 190_000)/8760
-450.0

# view the linearized objective function:
julia> MOI.get(model.solver, MOI.ObjectiveFunction{MOI.get(model.solver, MOI.ObjectiveFunctionType())}())
MathOptInterface.ScalarAffineFunction{Float64}(MathOptInterface.ScalarAffineTerm{Float64}[MathOptInterface.ScalarAffineTerm{Float64}(1.314e6, MathOptInterface.VariableIndex(8)), MathOptInterface.ScalarAffineTerm{Float64}(876000.0, MathOptInterface.VariableIndex(20)), MathOptInterface.ScalarAffineTerm{Float64}(87600.0, MathOptInterface.VariableIndex(25)), MathOptInterface.ScalarAffineTerm{Float64}(105120.0, MathOptInterface.VariableIndex(26)), MathOptInterface.ScalarAffineTerm{Float64}(131400.0, MathOptInterface.VariableIndex(27)), MathOptInterface.ScalarAffineTerm{Float64}(-1.752e6, MathOptInterface.VariableIndex(28)), MathOptInterface.ScalarAffineTerm{Float64}(40000.0, MathOptInterface.VariableIndex(29))], 0.0)

all variable indices: 8, 20, 25, 26, 27, 28, 29
7 total: x, y[1], λ, y[2], y[3], μ_up[y2], μ_up[y3]  (μ_lo's get multiplied by zero lower bounds)

# check the value of a term in the linearized objective function: 
it appears that variable 29 is "x"
julia> MOI.get(model.solver, MOI.VariablePrimal(), MathOptInterface.VariableIndex(29))
50.0
(coef is 40,000)

# probably 28 is lambda
julia> MOI.get(model.solver, MOI.VariablePrimal(), MathOptInterface.VariableIndex(28))
15.0
A_jn * w[j] = -8760*200 = -1.752e6

25 <-> y[1]
julia> MOI.get(model.solver, MOI.VariablePrimal(), MathOptInterface.VariableIndex(25))
50.0
coef is 8760 *10

26 <-> y[2]
julia> MOI.get(model.solver, MOI.VariablePrimal(), MathOptInterface.VariableIndex(26))
150.0
A_jn / V_jn * c_n = 8760 / 1 * 12 = 105120


rest of values are zero, which indicates that issue with mismatched objective values is not with value of duals in objective, but with coefficient(s)
objective value is off by -450*8760, but all of the coefs look good,
so maybe missing a term? maybe μ ⟂ (y - y_up) etc is not being held s.t. μ takes non zero value?

possible mu indices: 8, 20, 27

since 8760*150 = 1.314e6 MathOptInterface.VariableIndex(8) is probably the upper dual of y[2]
similarly since 8760*100 = 876000 MathOptInterface.VariableIndex(20) is probably the upper dual of y[3]

by process of elimination 
27 <-> y[3] = 0
and its coef should be 15*8760 = 131400

all coefficients check out

Nothing left to do but check the explicit linearization w/KKT:
=#
#=
the linearized single level model requires converting the y[1] ≤ x constraint to an equality, which
makes Ux + Vy = w:

[0; -1] x + [1 1 1 0; 1 0 0 1] [y; s] = [200; 0]

The KKT model is:

    ∇f_y + V^T λ + C^T μ = 0
    Ux + Vy = w
    Cy ≤ d
    where:
    y = [y[1], y[2], y[3], s]
    ∇f_y = [10; 12; 15; 0]  (last entry for s, the slack variable)
    V = [1 1 1 0; 1 0 0 1]
    C = [[1 1 1 1]; [-1 -1 -1 -1]]
=#



model = JuMP.Model(Gurobi.Optimizer)
set_optimizer_attribute(model, "MIPGap", 1e-2)

@variable(model, 0 <= y[i=1:3] <= 300, start = [50, 150, 0][i])
@variable(model, 0 <= x <= 250, start = 50)
@variable(model, 0 <= lambda <= 20, start = 15)

mu_up_bound = 10
mu_lo_bound = 0  # changing to negative lower bound gives correct lambda, but still wrong solution according to bilinear solution
@variables(model, begin
    mu_up_bound >= mu_1_up >= mu_lo_bound
    mu_up_bound >= mu_2_up >= mu_lo_bound
    mu_up_bound >= mu_3_up >= mu_lo_bound
    mu_up_bound >= mu_1_lo >= mu_lo_bound
    mu_up_bound >= mu_2_lo >= mu_lo_bound
    mu_up_bound >= mu_3_lo >= mu_lo_bound
    mu_up_bound >= mu_s_lo >= mu_lo_bound
    mu_up_bound >= mu_s_up >= mu_lo_bound
    mu_up_bound >= s >= mu_lo_bound
    lambda2
end)

@objective(model, Min, 40_000x + 87600*y[1] -8760*( lambda*200 - 12*y[2] - 15*y[3] - mu_2_up*150 - mu_3_up*100))

@constraint(model, b, y[1] + y[2] + y[3] == 200)
@constraint(model, ub1, y[1] - x + s == 0)  # remove this constraint here and in bilinear problem and get same results: 0 obj val, x=0, y[1]=200, y[2]=y[3]=0, lambda=10
@constraint(model, ub2, y[2] <= 150)
@constraint(model, ub3, y[3] <= 100)

#= 
    ∇f_y + V^T λ + C^T μ = 0
=#
@constraints(model, begin
    10 - lambda - lambda2 + mu_1_up == mu_1_lo 
    12 - lambda + mu_2_up == mu_2_lo  # want mu_2_up = 3 s.t. lambda = 15
    15 - lambda + mu_3_up == mu_3_lo
    0 - lambda2 + mu_s_up == mu_s_lo
end)
# if mu_1_up and mu_2_up are non zero then y[2] and y[3] would have to be at their upper bounds (150 and 100), but that's not possible given that y[3] >= 0 and their sum must be 200
# so try removing some complementary slackness to match bilinear solution
@constraint(model, [y[1]-300, mu_1_up] in MOI.SOS1([1.0, 2.0]))
@constraint(model, [y[2]-150, mu_2_up] in MOI.SOS1([1.0, 2.0]))
@constraint(model, [y[3]-100, mu_3_up] in MOI.SOS1([1.0, 2.0]))
@constraint(model, [y[1], mu_1_lo] in MOI.SOS1([1.0, 2.0]))
@constraint(model, [y[2], mu_2_lo] in MOI.SOS1([1.0, 2.0]))
@constraint(model, [y[3], mu_3_lo] in MOI.SOS1([1.0, 2.0]))
@constraint(model, [s, mu_s_lo] in MOI.SOS1([1.0, 2.0]))
# seems like maybe the mu variables are not constrained to be positive in BilevelJuMP ? have to add these constraints manually?

optimize!(model)

# check the linearization: (passes, which indicates that there is a bug in BilevelJuMP or Dualization)
@test value(40_000x + 8760*(10y[1]-lambda*y[1])) == objective_value(model)

@test value(x) ≈ 50 atol=1e-3 rtol=1e-2
@test value(y[1]) ≈ 50 atol=1e-3 rtol=1e-2
@test value(y[2]) ≈ 150 atol=1e-3 rtol=1e-2
@test value(y[3]) ≈ 0 atol=1e-3 rtol=1e-2
@test value(lambda) ≈ 15 atol=1e-3 rtol=1e-2
@test value(mu_2_up) ≈ 3 atol=1e-3 rtol=1e-2

@test value(40_000x + 8760*(10y[1]-lambda*y[1])) == -190000.0

#=
issue is that LL KKT model does not have the standard form (with slack variables), this might imply that we have to pass the standard form model to Dualization
test this hypothesis by passing in standard form LL model, then try creating LL model using standard_form method
=#


optimizer = Gurobi.Optimizer()
model = BilevelModel(()->optimizer, linearize_bilinear_upper_terms=true)
@variable(Upper(model), x, start = 50)
@variable(Lower(model), -1 <= y[i=1:3] <= 300, start = [50, 150, 0][i])
@variable(Lower(model), 300 >= s >= 0)

@constraint(Upper(model), lb0, x >= 0)
@constraint(Upper(model), ub0, x <= 250)

@objective(Lower(model), Min, 10y[1] + 12y[2] + 15y[3])

@constraint(Lower(model), b, y[1] + y[2] + y[3] == 200)
@constraint(Lower(model), b2, y[1] - x + s == 0)
@constraint(Lower(model), ub2, y[2] <= 150)
@constraint(Lower(model), ub3, y[3] <= 100)
@constraint(Lower(model), ub4, y[3] <= 300)
@constraint(Lower(model), lb[i=1:3], y[i] >= 0)
@variable(Upper(model), 0 <= lambda <= 20, DualOf(b), start = 15)


@objective(Upper(model), Min, 40_000x + 8760*(10y[1]-lambda*y[1]))

# BilevelJuMP.set_dual_lower_bound(ub4, 0)
# BilevelJuMP.set_dual_lower_bound(ub3, 0)
# BilevelJuMP.set_dual_lower_bound(ub2, 0)
# for cref in lb
#     BilevelJuMP.set_dual_lower_bound(cref, 0)
# end

optimize!(model)


@test value(x) ≈ 50 atol=1e-3 rtol=1e-2
@test value.(y) ≈ [50, 150, 0] atol=1e-3 rtol=1e-2
@test value(lambda) ≈ 15 atol=1e-3 rtol=1e-2
@test value(40_000x + 8760*(10y[1]-lambda*y[1])) ≈ -190000.0 atol=1e-3 rtol=1e-2
@test value(40_000x + 8760*(10y[1]-lambda*y[1])) ≈ objective_value(model) atol=1e-3 rtol=1e-2

#=


correct variable values but incorrect objective function

    all linearizations:
julia> push!(linearizations, MOI.ScalarAffineTerm(-A_jn / V_jn * upp_bound, lower_dual_idxmap[upp_dual]))
7-element Vector{MathOptInterface.ScalarAffineTerm}:
 MathOptInterface.ScalarAffineTerm{Float64}(-1.752e6, MathOptInterface.VariableIndex(6))
 MathOptInterface.ScalarAffineTerm{Float64}(87600.0, MathOptInterface.VariableIndex(2))
 MathOptInterface.ScalarAffineTerm{Float64}(-0.0, MathOptInterface.VariableIndex(10))
 MathOptInterface.ScalarAffineTerm{Float64}(2.628e6, MathOptInterface.VariableIndex(17))
 MathOptInterface.ScalarAffineTerm{Float64}(131400.0, MathOptInterface.VariableIndex(4))
 MathOptInterface.ScalarAffineTerm{Float64}(-0.0, MathOptInterface.VariableIndex(12))
 MathOptInterface.ScalarAffineTerm{Float64}(876000.0, MathOptInterface.VariableIndex(19))

MOI.get(model.solver, MOI.ObjectiveFunction{MOI.get(model.solver, MOI.ObjectiveFunctionType())}())
MathOptInterface.ScalarAffineFunction{Float64}(MathOptInterface.ScalarAffineTerm{Float64}[
    MathOptInterface.ScalarAffineTerm{Float64}(1.314e6, MathOptInterface.VariableIndex(12)), 
    MathOptInterface.ScalarAffineTerm{Float64}(876000.0, MathOptInterface.VariableIndex(24)), 
    MathOptInterface.ScalarAffineTerm{Float64}(87600.0, MathOptInterface.VariableIndex(29)), 
    MathOptInterface.ScalarAffineTerm{Float64}(105120.0, MathOptInterface.VariableIndex(30)), 
    MathOptInterface.ScalarAffineTerm{Float64}(131400.0, MathOptInterface.VariableIndex(31)), 
    MathOptInterface.ScalarAffineTerm{Float64}(-1.752e6, MathOptInterface.VariableIndex(33)), 
    MathOptInterface.ScalarAffineTerm{Float64}(40000.0, MathOptInterface.VariableIndex(34))], 0.0)


Using wrong indices ?

objective value is STILL off by -450*8760, which is mu_2_up * it's coefs (3*150*-8760)
so probably not getting right variable index for mu_2_up (and possibly others)

MOI.get(model.solver, MOI.VariablePrimal(), MathOptInterface.VariableIndex(6))
5.000000000235845  maybe lambda2 ?

julia> MOI.get(model.solver, MOI.VariablePrimal(), MathOptInterface.VariableIndex(35)) # last variable index is 35
-5.000000000235845

julia> MOI.get(model.solver, MOI.VariablePrimal(), MathOptInterface.VariableIndex(8))
-3.0000000002358496  maybe mu_2_up (both have sign changes)
=#