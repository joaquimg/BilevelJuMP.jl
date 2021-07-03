using BilevelJuMP
using Ipopt
using JuMP
using Test

model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-9))


@variable(Upper(model), y, start = 8 / 15)
@variable(Lower(model), x, start = 3.5 * 8 / 15)
@objective(Upper(model), Min, 3x + y)
@constraints(Upper(model), begin
    u1, x <= 5
    u2, y <= 8
    u3, y >= 0
end)

@objective(Lower(model), Min, -x)
@constraint(Lower(model), l1,  x +  y <= 8)
@constraint(Lower(model), l2, 4x +  y >= 8)
@constraint(Lower(model), l3, 2x +  y <= 13)
@constraint(Lower(model), l4, 2x - 7y <= 0)


function _build_single_model(
    model::BilevelModel,
)
    upper = JuMP.backend(model.upper)
    lower = JuMP.backend(model.lower)
    link = JuMP.index(model.link)


    return _build_single_model(upper, lower, link)
end 

function _build_single_model(
    upper::MOI.ModelLike, 
    lower::MOI.ModelLike, 
    # A dictionary that maps variables in the upper to variables in the lower
    link::Dict{MOI.VariableIndex,MOI.VariableIndex},
)
    # A new model to build
    model = MOI.Utilities.Model{Float64}()
    # Create a copy of the upper model
    index_map = MOI.copy_to(model, upper)
    # TODO ... add the lower problem to `model`
    return model, upper_variables, lower_variables, lower_objective
end


optimize!(model)

# Automated testing

@test objective_value(model) ≈ 3 * (3.5 * 8 / 15) + (8 / 15) atol=1e-6
@test BilevelJuMP.lower_objective_value(model) ≈ -3.5 * 8 / 15 atol=1e-6
@test objective_value(Lower(model)) ≈ -3.5 * 8 / 15 atol=1e-6
@test value(x) ≈ 3.5 * 8 / 15 atol=1e-6
@test value(y) ≈ 8 / 15 atol=1e-6
@test value(u1) ≈ 3.5 * 8 / 15 atol=1e-6
@test value(l1) ≈ 4.5 * 8 / 15 atol=1e-6
@test dual(l1) ≈ [0] atol=1e-6
@test dual(l3) ≈ [0] atol=1e-6

# TODO: why are these commented out?    #src
# @test dual(l2) #≈ [0] atol=atol       #src
# @test dual(l4) #≈ [0] atol=atol       #src
