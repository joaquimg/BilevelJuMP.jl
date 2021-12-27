
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
# this problem violates condition 1? Ujm = 0 ∀ j ∈ J_U, ∀ m ∈ M
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
# this problem violates condition 1? Ujm = 0 ∀ j ∈ J_U, ∀ m ∈ M
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


@test value(40_000x + 8760*(10y[1]-lambda*y[1])) == -190000.0
# TODO why doesn't objective_value match the Upper objective value?
# @test objective_value(model) ≈ -190_000 atol=1e-1 rtol=1e-2
@test value(x) ≈ 50 atol=1e-3 rtol=1e-2
@test value.(y) ≈ [50, 150, 0] atol=1e-3 rtol=1e-2
@test value(lambda) ≈ 15 atol=1e-3 rtol=1e-2
